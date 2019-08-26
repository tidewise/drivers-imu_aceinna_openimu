#include <imu_aceinna_openimu/Protocol.hpp>
#include <imu_aceinna_openimu/Endianness.hpp>
#include <imu_aceinna_openimu/EKFWithCovariance.hpp>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/RigidBodyAcceleration.hpp>

using namespace std;
using namespace imu_aceinna_openimu;
using endianness::decode;

ORIENTATION_AXIS decodeOrientationAxis(string axis) {
    int result;
    if (axis[1] == 'X') {
        result = ORIENTATION_AXIS_PLUS_X;
    }
    else if (axis[1] == 'Y') {
        result = ORIENTATION_AXIS_PLUS_Y;
    }
    else if (axis[1] == 'Z') {
        result = ORIENTATION_AXIS_PLUS_Z;
    }
    else {
        throw std::invalid_argument("unexpected axis letter in '" + axis +
                                    "', expected X, Y or Z");
    }

    if (axis[0] == '-') {
        result += 1;
    }
    else if (axis[0] != '+') {
        throw std::invalid_argument("unexpected axis direction in '" + axis +
                                    "', expected + or -");
    }
    return static_cast<ORIENTATION_AXIS>(result);
}

Configuration::Orientation protocol::decodeOrientationString(string orientation)
{
    Configuration::Orientation ret;
    ret.forward = decodeOrientationAxis(orientation.substr(0, 2));
    ret.right = decodeOrientationAxis(orientation.substr(2, 2));
    ret.down = decodeOrientationAxis(orientation.substr(4, 2));
    return ret;
}

static std::string encodeOrientationAxis(ORIENTATION_AXIS axis) {
    switch(axis) {
        case ORIENTATION_AXIS_PLUS_X: return "+X";
        case ORIENTATION_AXIS_MINUS_X: return "-X";
        case ORIENTATION_AXIS_PLUS_Y: return "+Y";
        case ORIENTATION_AXIS_MINUS_Y: return "-Y";
        case ORIENTATION_AXIS_PLUS_Z: return "+Z";
        case ORIENTATION_AXIS_MINUS_Z: return "-Z";
        default:
            throw std::invalid_argument("encodeOrientationAxis: invalid axis");
    }
}

int protocol::extractPacket(uint8_t const* buffer, int bufferSize)
{
    if (bufferSize < MIN_PACKET_SIZE) {
        return 0;
    }

    int packetStart = 0;
    for (packetStart = 0; packetStart < bufferSize; ++packetStart) {
        if (buffer[packetStart] == PACKET_START_MARKER) {
            if (packetStart + 1 == bufferSize) {
                return -packetStart;
            }
            else if (buffer[packetStart + 1] == PACKET_START_MARKER) {
                break;
            }
        }
    }

    if (packetStart != 0) {
        return -packetStart;
    }

    // We are guaranteed to have a two-byte header and enough bytes to have
    // something that could be a packet. Check it out
    int payloadSize = buffer[4];
    int packetSize = payloadSize + PACKET_OVERHEAD;
    if (bufferSize < packetSize) {
        return 0;
    }

    uint16_t crc0 = buffer[packetSize - 2];
    uint16_t crc1 = buffer[packetSize - 1];
    uint16_t crc = (crc0 << 8) + crc1;

    uint16_t expectedCRC = protocol::crc(buffer + 2, buffer + packetSize - 2);
    if (expectedCRC == crc) {
        return packetSize;
    }
    else {
        return -1;
    }
}

uint16_t protocol::crc(uint8_t const* begin, uint8_t const* end)
{
    uint32_t crc = 0x1D0F;
    for (auto it = begin; it != end; ++it) {
        crc = crc ^ (static_cast<uint16_t>(*it) << 8);
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = (crc << 1);
        }
        crc = crc & 0xffff;
    }

    return crc;
}

uint8_t* protocol::formatPacket(uint8_t* buffer, char const* code,
                                uint8_t const* payload, int size)
{
    buffer[0] = protocol::PACKET_START_MARKER;
    buffer[1] = protocol::PACKET_START_MARKER;
    buffer[2] = code[0];
    buffer[3] = code[1];
    buffer[4] = size;
    memcpy(&buffer[5], payload, size);

    uint8_t* payloadEnd = &buffer[5] + size;
    uint16_t crc = protocol::crc(&buffer[2], payloadEnd);
    payloadEnd[0] = (crc >> 8) & 0xFF;
    payloadEnd[1] = (crc >> 0) & 0xFF;

    return buffer + PACKET_OVERHEAD + size;
}

uint8_t* protocol::queryDeviceID(uint8_t* buffer)
{
    return protocol::formatPacket(buffer, "pG", nullptr, 0);
}

std::string protocol::parseDeviceID(uint8_t const* payload, int size)
{
    return std::string(payload, payload + size);
}

uint8_t* protocol::queryAppVersion(uint8_t* buffer)
{
    return protocol::formatPacket(buffer, "gV", nullptr, 0);
}

std::string protocol::parseAppVersion(uint8_t const* payload, int size)
{
    return std::string(payload, payload + size);
}

uint8_t* protocol::queryConfiguration(uint8_t* buffer)
{
    return protocol::formatPacket(buffer, "gA", nullptr, 0);
}

Configuration protocol::parseConfiguration(uint8_t const* buffer, int bufferSize)
{
    if (bufferSize < 96) {
        throw std::invalid_argument("buffer size for message gA (configuration) "\
                                    "smaller than 96 bytes");
    }
    uint8_t const* end = buffer + bufferSize;
    Configuration ret;
    ret.periodic_packet_type = string(
        reinterpret_cast<char const*>(buffer + 24),
        reinterpret_cast<char const*>(buffer + 26)
    );
    uint8_t const* cursor = buffer;

    int64_t values[3];
    cursor = decode<int64_t>(buffer + 32, values[0], end);
    cursor = decode<int64_t>(cursor, values[1], end);
    cursor = decode<int64_t>(cursor, values[2], end);
    ret.periodic_packet_rate = values[0];
    ret.acceleration_low_pass_filter = values[1];
    ret.angular_velocity_low_pass_filter = values[2];

    string orientation(
        reinterpret_cast<char const*>(buffer + 56),
        reinterpret_cast<char const*>(buffer + 62)
    );
    ret.orientation = decodeOrientationString(orientation);

    // GPS parameters, standard INS app
    int64_t protocol, baudrate;
    cursor = decode(buffer + 64, baudrate, end);
    cursor = decode(cursor, protocol, end);
    if (protocol < -1 || protocol > GPS_LAST_KNOWN_PROTOCOL) {
        throw std::invalid_argument("got invalid GPS protocol");
    }
    ret.gps_protocol = static_cast<GPSProtocol>(protocol);
    ret.gps_baud_rate = baudrate;

    // Read but ignore
    double magneticCalibration[4];
    for (int i = 0; i < 4; ++i)
        cursor = decode(cursor, magneticCalibration[i], end);

    // Flags
    uint64_t sensorsUsed;
    cursor = decode(cursor, sensorsUsed, end);
    ret.use_magnetometers = sensorsUsed & 0x1;
    ret.use_gps = sensorsUsed & 0x2;
    ret.use_gps_course_as_heading = sensorsUsed & 0x4;
    return ret;
}

uint8_t* protocol::queryConfigurationParameter(uint8_t* buffer, int index) {
    uint8_t payload[4];
    endianness::encode<uint32_t>(payload, index);
    return formatPacket(buffer, "gP", payload, 4);
}

static void validateConfigurationParameter(uint8_t const* buffer, int bufferSize,
                                           int expectedIndex) {
    if (bufferSize != 12) {
        throw std::invalid_argument("unexpected buffer size for gP response, "
                                    "expected 12 but got " + to_string(bufferSize));
    }
    int32_t actualIndex;
    endianness::decode<int32_t>(buffer, actualIndex);
    if (actualIndex != expectedIndex) {
        throw std::invalid_argument("was expecting a read of configuration parameter " +
                                    to_string(expectedIndex) + " but got " +
                                    to_string(actualIndex));
    }
}

template<> Configuration::Orientation protocol::parseConfigurationParameter<Configuration::Orientation>(
    uint8_t* buffer, int bufferSize, int expectedIndex) {
    validateConfigurationParameter(buffer, bufferSize, expectedIndex);
    string orientation = string(
        reinterpret_cast<char const*>(buffer + 4),
        reinterpret_cast<char const*>(buffer + 12)
    );
    return decodeOrientationString(orientation);
}

template<> string protocol::parseConfigurationParameter<string>(
    uint8_t* buffer, int bufferSize, int expectedIndex) {
    validateConfigurationParameter(buffer, bufferSize, expectedIndex);
    return string(
        reinterpret_cast<char const*>(buffer + 4),
        reinterpret_cast<char const*>(buffer + 12)
    );
}

template<> int64_t protocol::parseConfigurationParameter<int64_t>(
    uint8_t* buffer, int bufferSize, int expectedIndex) {
    validateConfigurationParameter(buffer, bufferSize, expectedIndex);
    int64_t value;
    endianness::decode<int64_t>(buffer + 4, value);
    return value;
}

template<>
uint8_t* protocol::writeConfiguration<int64_t>(uint8_t* buffer, int index, int64_t value) {
    uint8_t payload[12];
    endianness::encode<uint32_t>(payload, index);
    endianness::encode<int64_t>(payload + 4, value);
    return formatPacket(buffer, "uP", payload, 12);
}

template<>
uint8_t* protocol::writeConfiguration<std::string>(uint8_t* buffer, int index, std::string value) {
    if (value.length() > 8) {
        throw std::invalid_argument("cannot encode strings longer than 8 characters");
    }

    uint8_t payload[12];
    memset(payload, 0, 12);
    endianness::encode<uint32_t>(payload, index);
    memcpy(payload + 4, &(value.at(0)), value.length());
    return formatPacket(buffer, "uP", payload, 12);
}

uint8_t* protocol::writeConfiguration(
    uint8_t* buffer, int index, Configuration::Orientation orientation)
{
    string encoded = encodeOrientationAxis(orientation.forward) +
                     encodeOrientationAxis(orientation.right) +
                     encodeOrientationAxis(orientation.down);
    return writeConfiguration(buffer, index, encoded);
}

protocol::WriteStatus protocol::parseWriteConfigurationStatus(
    uint8_t* buffer, int bufferSize) {
    if (bufferSize != 4) {
        throw std::invalid_argument(
            "unexpected reply size for uP (write configuration parameter), "
            "expected 4 bytes, got " + to_string(bufferSize));
    }

    int32_t ret;
    endianness::decode<int32_t>(buffer, ret);
    if (ret == 0) {
        return WRITE_STATUS_OK;
    }
    else if (ret == -1) {
        return WRITE_STATUS_INVALID_INDEX;
    }
    else if (ret == -2) {
        return WRITE_STATUS_INVALID_VALUE;
    }
    else {
        return WRITE_STATUS_UNKNOWN;
    }
}

uint8_t* protocol::queryConfigurationSave(uint8_t* buffer) {
    return formatPacket(buffer, "sC", nullptr, 0);
}

uint8_t* protocol::queryRestoreDefaultConfiguration(uint8_t* buffer) {
    return formatPacket(buffer, "rD", nullptr, 0);
}

uint8_t* protocol::queryReset(uint8_t* buffer) {
    return formatPacket(buffer, "rS", nullptr, 0);
}

uint8_t* protocol::queryJumpToBootloader(uint8_t* buffer) {
    return formatPacket(buffer, "JI", nullptr, 0);
}

uint8_t* protocol::queryJumpToApp(uint8_t* buffer) {
    return formatPacket(buffer, "JA", nullptr, 0);
}

uint8_t* protocol::queryAppBlockWrite(uint8_t* buffer, uint32_t address,
                                      uint8_t const* blockData, int blockSize) {
    if (blockSize > MAX_APP_BLOCK_SIZE) {
        throw std::invalid_argument("max app block is 240 bytes");
    }

    uint8_t payload[256];
    payload[0] = (address >> 24) & 0xFF;
    payload[1] = (address >> 16) & 0xFF;
    payload[2] = (address >> 8) & 0xFF;
    payload[3] = (address >> 0) & 0xFF;
    payload[4] = blockSize;
    memcpy(&payload[5], blockData, blockSize);
    return formatPacket(buffer, "WA", payload, blockSize + 5);
}

uint8_t* protocol::queryStatus(uint8_t* buffer)
{
    return formatPacket(buffer, "gS", nullptr, 0);
}

Status protocol::parseStatus(uint8_t const* buffer, int size)
{
    uint8_t const* end = buffer + size;
    uint8_t const* cursor = buffer;

    uint32_t time, last_gps_message, last_gps, last_gps_velocity, gps_rx;
    uint16_t gps_overflows;
    uint8_t temperature_C;
    cursor = endianness::decode(cursor, time, end);
    cursor = endianness::decode(cursor, last_gps_message, end);
    cursor = endianness::decode(cursor, last_gps, end);
    cursor = endianness::decode(cursor, last_gps_velocity, end);
    cursor = endianness::decode(cursor, gps_rx, end);
    cursor = endianness::decode(cursor, gps_overflows, end);
    cursor = endianness::decode(cursor, temperature_C, end);

    if (cursor != end) {
        throw std::invalid_argument("received status structure bigger than expected");
    }

    Status ret;
    ret.time = base::Time::fromMilliseconds(time);
    ret.gps_rx = gps_rx;
    ret.gps_overflows = gps_overflows;
    ret.last_gps_message = base::Time::fromMilliseconds(last_gps_message);
    ret.last_good_gps = base::Time::fromMilliseconds(last_gps);
    ret.last_usable_gps_velocity = base::Time::fromMilliseconds(last_gps_velocity);
    ret.temperature = base::Temperature::fromCelsius(temperature_C);
    return ret;
}

EKFWithCovariance protocol::parseEKFWithCovariance(uint8_t const* buffer, int bufferSize) {
    uint8_t const* end = buffer + bufferSize;

    base::samples::RigidBodyState rbs;
    base::samples::RigidBodyAcceleration rba;

    uint8_t const* cursor = buffer;
    uint32_t time_gps_itow;
    cursor = endianness::decode<uint32_t>(buffer, time_gps_itow, end);
    rbs.time = rba.time = base::Time::fromMilliseconds(time_gps_itow);

    float values[24];
    for (int i = 0; i < 24; ++i) {
        cursor = endianness::decode<float>(cursor, values[i], end);
    }
    double lat_lon_alt[3];
    for (int i = 0; i < 3; ++i) {
        cursor = endianness::decode<double>(cursor, lat_lon_alt[i], end);
    }
    float pos_covariance[3];
    for (int i = 0; i < 3; ++i) {
        cursor = endianness::decode<float>(cursor, pos_covariance[i], end);
    }
    uint8_t status_byte = *cursor++;

    if (cursor != end) {
        throw std::invalid_argument(
            "too many bytes in buffer: got " + to_string(bufferSize) +
            ", expected " + to_string(cursor - buffer));
    }

    static const double deg2rad = M_PI / 180.0;
    static const double deg2rad_square = deg2rad * deg2rad;

    rbs.orientation =
        Eigen::AngleAxisd(values[2] * deg2rad, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(values[1] * deg2rad, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(values[0] * deg2rad, Eigen::Vector3d::UnitX());
    rbs.cov_orientation = Eigen::DiagonalMatrix<double, 3>(
        values[3] * deg2rad_square,
        values[4] * deg2rad_square,
        values[5] * deg2rad_square
    );

    FilterState state;
    state.mode = static_cast<FilterMode>(status_byte & 0x7);
    state.status = status_byte >> 3;

    rba.acceleration = Eigen::Vector3d(values[6], values[7], values[8]);
    rba.cov_acceleration = Eigen::DiagonalMatrix<double, 3>(
        values[9], values[10], values[11]);

    rbs.angular_velocity = Eigen::Vector3d(
        values[12], values[13], values[14]) * deg2rad;
    rbs.cov_angular_velocity = Eigen::DiagonalMatrix<double, 3>(
        values[15], values[16], values[17]) * deg2rad_square;

    EKFWithCovariance result;
    if (state.mode == OPMODE_INS) {
        rbs.velocity = Eigen::Vector3d(
            values[18], values[19], values[20]);
        rbs.cov_velocity = Eigen::DiagonalMatrix<double, 3>(
            values[21], values[22], values[23]);

        rbs.cov_position = Eigen::DiagonalMatrix<double, 3>(
            pos_covariance[0], pos_covariance[1], pos_covariance[2]);
        result.latitude = base::Angle::fromDeg(lat_lon_alt[0]);
        result.longitude = base::Angle::fromDeg(lat_lon_alt[1]);
        rbs.position.z() = lat_lon_alt[2];
    }

    result.rbs = rbs;
    result.rba = rba;
    result.filter_state = state;
    return result;
}

