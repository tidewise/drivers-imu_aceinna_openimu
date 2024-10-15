#include <base/samples/RigidBodyAcceleration.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <cstring>
#include <imu_aceinna_openimu/Endianness.hpp>
#include <imu_aceinna_openimu/PeriodicUpdate.hpp>
#include <imu_aceinna_openimu/Protocol.hpp>
#include <iostream>
#include <stdexcept>

using namespace std;
using namespace imu_aceinna_openimu;
using endianness::decode;

int protocol::extractPacket(uint8_t const* buffer, int bufferSize)
{
    if (bufferSize < MIN_PACKET_SIZE) {
        return 0;
    }

    int packetStart = 0;
    for (packetStart = 0; packetStart < bufferSize; ++packetStart) {
        if (buffer[packetStart] == PACKET_START_MARKER) {
            if (packetStart + 1 == bufferSize) {
                return -packetStart; // remove all bytes before the packet start marker
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

uint8_t* protocol::formatPacket(uint8_t* buffer,
    char const* code,
    uint8_t const* payload,
    int size)
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
        throw std::invalid_argument("buffer size for message gA (configuration) "
                                    "smaller than 96 bytes");
    }
    uint8_t const* end = buffer + bufferSize;
    Configuration ret;
    ret.periodic_packet_type = string(reinterpret_cast<char const*>(buffer + 24),
        reinterpret_cast<char const*>(buffer + 26));
    uint8_t const* cursor = buffer;

    int64_t values[3];
    cursor = decode<int64_t>(buffer + 32, values[0], end);
    cursor = decode<int64_t>(cursor, values[1], end);
    cursor = decode<int64_t>(cursor, values[2], end);
    ret.periodic_packet_rate = values[0];
    ret.acceleration_low_pass_filter = values[1];
    ret.angular_velocity_low_pass_filter = values[2];

    ret.orientation = string(reinterpret_cast<char const*>(buffer + 56),
        reinterpret_cast<char const*>(buffer + 62));

    // GPS parameters, standard INS app
    int64_t protocol, baudrate;
    cursor = decode(buffer + 64, baudrate, end);
    cursor = decode(cursor, protocol, end);
    if (protocol < -1 || protocol > GPS_LAST_KNOWN_PROTOCOL) {
        throw std::invalid_argument("got invalid GPS protocol");
    }
    ret.gps_protocol = static_cast<GPSProtocol>(protocol);
    ret.gps_baud_rate = baudrate;

    cursor = decode(cursor, ret.hard_iron[0], end);
    cursor = decode(cursor, ret.hard_iron[1], end);
    cursor = decode(cursor, ret.soft_iron_ratio, end);
    cursor = decode(cursor, ret.soft_iron_angle, end);

    double lever_arm[3];
    for (int i = 0; i < 3; ++i) {
        cursor = decode(cursor, lever_arm[i], end);
    }
    ret.lever_arm = base::Vector3d(lever_arm[0], lever_arm[1], lever_arm[2]);

    double point_of_interest[3];
    for (int i = 0; i < 3; ++i) {
        cursor = decode(cursor, point_of_interest[i], end);
    }
    ret.point_of_interest =
        base::Vector3d(point_of_interest[0], point_of_interest[1], point_of_interest[2]);

    double rtk_heading2mag_heading;
    if (cursor != end) {
        cursor = decode(cursor, rtk_heading2mag_heading, end);
        ret.rtk_heading2mag_heading = base::Angle::fromRad(rtk_heading2mag_heading);
    }

    return ret;
}

uint8_t* protocol::queryConfigurationParameter(uint8_t* buffer, int index)
{
    uint8_t payload[4];
    endianness::encode<uint32_t>(payload, index);
    return formatPacket(buffer, "gP", payload, 4);
}

static void validateConfigurationParameter(uint8_t const* buffer,
    int bufferSize,
    int expectedIndex)
{
    if (bufferSize != 12) {
        throw std::invalid_argument("unexpected buffer size for gP response, "
                                    "expected 12 but got " +
                                    to_string(bufferSize));
    }
    int32_t actualIndex;
    endianness::decode<int32_t>(buffer, actualIndex);
    if (actualIndex != expectedIndex) {
        throw std::invalid_argument("was expecting a read of configuration parameter " +
                                    to_string(expectedIndex) + " but got " +
                                    to_string(actualIndex));
    }
}

template <>
string protocol::parseConfigurationParameter<string>(uint8_t* buffer,
    int bufferSize,
    int expectedIndex)
{
    validateConfigurationParameter(buffer, bufferSize, expectedIndex);
    for (size_t string_end = 4; string_end < 12; ++string_end) {
        if (buffer[string_end] == 0) {
            return string(reinterpret_cast<char const*>(buffer + 4),
                reinterpret_cast<char const*>(buffer + string_end));
        }
    }
    return string(reinterpret_cast<char const*>(buffer + 4),
        reinterpret_cast<char const*>(buffer + 12));
}

template <>
int64_t protocol::parseConfigurationParameter<int64_t>(uint8_t* buffer,
    int bufferSize,
    int expectedIndex)
{
    validateConfigurationParameter(buffer, bufferSize, expectedIndex);
    int64_t value;
    endianness::decode<int64_t>(buffer + 4, value);
    return value;
}

template <>
double protocol::parseConfigurationParameter<double>(uint8_t* buffer,
    int bufferSize,
    int expectedIndex)
{
    validateConfigurationParameter(buffer, bufferSize, expectedIndex);
    int64_t value;
    endianness::decode<int64_t>(buffer + 4, value);
    return reinterpret_cast<double&>(value);
}

template <>
uint8_t* protocol::writeConfiguration<int64_t>(uint8_t* buffer, int index, int64_t value)
{
    uint8_t payload[12];
    endianness::encode<uint32_t>(payload, index);
    endianness::encode<int64_t>(payload + 4, value);
    return formatPacket(buffer, "uP", payload, 12);
}

template <>
uint8_t* protocol::writeConfiguration<double>(uint8_t* buffer, int index, double value)
{
    uint8_t payload[12];
    endianness::encode<uint32_t>(payload, index);
    endianness::encode<int64_t>(payload + 4, reinterpret_cast<int64_t&>(value));
    return formatPacket(buffer, "uP", payload, 12);
}

template <>
uint8_t* protocol::writeConfiguration<std::string>(uint8_t* buffer,
    int index,
    std::string value)
{
    if (value.length() > 8) {
        throw std::invalid_argument("cannot encode strings longer than 8 characters");
    }

    uint8_t payload[12];
    memset(payload, 0, 12);
    endianness::encode<uint32_t>(payload, index);
    memcpy(payload + 4, &(value.at(0)), value.length());
    return formatPacket(buffer, "uP", payload, 12);
}

protocol::WriteStatus protocol::parseWriteConfigurationStatus(uint8_t* buffer,
    int bufferSize)
{
    if (bufferSize != 4) {
        throw std::invalid_argument(
            "unexpected reply size for uP (write configuration parameter), "
            "expected 4 bytes, got " +
            to_string(bufferSize));
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

uint8_t* protocol::queryConfigurationSave(uint8_t* buffer)
{
    return formatPacket(buffer, "sC", nullptr, 0);
}

uint8_t* protocol::queryRestoreDefaultConfiguration(uint8_t* buffer)
{
    return formatPacket(buffer, "rD", nullptr, 0);
}

uint8_t* protocol::queryReset(uint8_t* buffer)
{
    return formatPacket(buffer, "rS", nullptr, 0);
}

uint8_t* protocol::queryJumpToBootloader(uint8_t* buffer)
{
    return formatPacket(buffer, "JI", nullptr, 0);
}

uint8_t* protocol::queryJumpToApp(uint8_t* buffer)
{
    return formatPacket(buffer, "JA", nullptr, 0);
}

uint8_t* protocol::queryAppBlockWrite(uint8_t* buffer,
    uint32_t address,
    uint8_t const* blockData,
    int blockSize)
{
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

FilterState status_byte2filter_state(uint8_t status_byte)
{
    FilterState state;
    state.mode = static_cast<FilterMode>(status_byte & 0x7);
    state.status = status_byte >> 3;
    return state;
}

Status protocol::parseStatus(uint8_t const* buffer, int size)
{
    uint8_t const* end = buffer + size;
    uint8_t const* cursor = buffer;

    uint32_t time, ext_periodic_overflow, gps_updates, last_gps_message, last_gps,
        last_gps_velocity, gps_rx;
    uint16_t gps_overflows, hdop;
    uint8_t temperature_C, status_byte;
    cursor = endianness::decode(cursor, time, end);
    cursor = endianness::decode(cursor, ext_periodic_overflow, end);
    cursor = endianness::decode(cursor, gps_updates, end);
    cursor = endianness::decode(cursor, last_gps_message, end);
    cursor = endianness::decode(cursor, last_gps, end);
    cursor = endianness::decode(cursor, last_gps_velocity, end);
    cursor = endianness::decode(cursor, gps_rx, end);
    cursor = endianness::decode(cursor, gps_overflows, end);
    cursor = endianness::decode(cursor, hdop, end);
    cursor = endianness::decode(cursor, temperature_C, end);
    cursor = endianness::decode(cursor, status_byte, end);

    if (cursor != end) {
        throw std::invalid_argument("received status structure bigger than expected");
    }

    Status ret;
    ret.time = base::Time::fromMilliseconds(time);
    ret.extended_periodic_packets_overflow = ext_periodic_overflow;
    ret.gps_updates = gps_updates;
    ret.gps_rx = gps_rx;
    ret.gps_overflows = gps_overflows;
    ret.last_gps_message = base::Time::fromMilliseconds(last_gps_message);
    ret.last_good_gps = base::Time::fromMilliseconds(last_gps);
    ret.last_usable_gps_velocity = base::Time::fromMilliseconds(last_gps_velocity);
    ret.hdop = static_cast<float>(hdop) / 10;
    ret.temperature = base::Temperature::fromCelsius(temperature_C);
    ret.filter_state = status_byte2filter_state(status_byte);
    return ret;
}

template <typename H> H protocol::covarianceNED2NWU(H neu)
{
    neu = RotNED2NWU * neu * RotNWU2NED;
    return neu;
}

PeriodicUpdate protocol::parseE2Output(uint8_t const* buffer, int bufferSize)
{
    uint8_t const* end = buffer + bufferSize;

    base::samples::RigidBodyState rbs;
    base::samples::RigidBodyAcceleration rba;

    uint8_t const* cursor = buffer;
    uint32_t time_ms;
    cursor = endianness::decode<uint32_t>(buffer, time_ms, end);
    cursor += 8; // skip the time-as-double field

    rbs.time = rba.time = base::Time::fromMilliseconds(time_ms);

    float values[21];
    for (int i = 0; i < 21; ++i) {
        cursor = endianness::decode<float>(cursor, values[i], end);
    }
    double lat_lon_alt[3];
    for (int i = 0; i < 3; ++i) {
        cursor = endianness::decode<double>(cursor, lat_lon_alt[i], end);
    }
    uint8_t op_mode = *cursor++;
    uint8_t lin_accel_switch = *cursor++;
    uint8_t turn_switch = *cursor++;

    if (cursor != end) {
        throw std::invalid_argument("too many bytes in buffer: got " +
                                    to_string(bufferSize) + ", expected " +
                                    to_string(cursor - buffer));
    }

    static const double deg2rad = M_PI / 180.0;
    static const double g2si = 9.80665;

    rbs.orientation = Eigen::AngleAxisd(values[2] * deg2rad, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(values[1] * deg2rad, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(values[0] * deg2rad, Eigen::Vector3d::UnitX());
    rbs.orientation = valueNED2NWU(rbs.orientation);

    rba.acceleration = Eigen::Vector3d(values[3], values[4], values[5]) * g2si;
    rbs.angular_velocity = Eigen::Vector3d(values[9], values[10], values[11]) * deg2rad;

    PeriodicUpdate result;
    if (op_mode == OPMODE_INS) {
        rbs.velocity = Eigen::Vector3d(values[15], -values[16], -values[17]);

        result.latitude = base::Angle::fromDeg(lat_lon_alt[0]);
        result.longitude = base::Angle::fromDeg(lat_lon_alt[1]);
        rbs.position.z() = lat_lon_alt[2];
    }

    result.rbs = rbs;
    result.rba = rba;

    FilterState state;
    state.mode = static_cast<FilterMode>(op_mode);
    state.status = 0;
    if (lin_accel_switch) {
        state.status |= LINEAR_ACCELERATION;
    }
    if (turn_switch) {
        state.status |= TURN_SWITCH;
    }
    result.filter_state = state;
    return result;
}

PeriodicUpdate protocol::parseE4Output(uint8_t const* buffer, int bufferSize)
{
    uint8_t const* end = buffer + bufferSize;

    base::samples::RigidBodyState rbs;
    base::samples::RigidBodyAcceleration rba;

    uint8_t const* cursor = buffer;
    uint32_t time_ms;
    cursor = endianness::decode<uint32_t>(buffer, time_ms, end);
    uint8_t filterFlags = cursor[0];
    ++cursor;

    float values[10];
    for (int i = 0; i < 10; ++i) {
        cursor = endianness::decode<float>(cursor, values[i], end);
    }
    double lat_lon_alt[3];
    for (int i = 0; i < 3; ++i) {
        cursor = endianness::decode<double>(cursor, lat_lon_alt[i], end);
    }
    float magInfo[7];
    for (int i = 0; i < 7; ++i) {
        cursor = endianness::decode<float>(cursor, magInfo[i], end);
    }

    static const double deg2rad = M_PI / 180.0;

    auto time = base::Time::fromMilliseconds(time_ms);

    FilterState state;
    state.time = time;
    state.mode = static_cast<FilterMode>(filterFlags & 0x7);
    state.status = filterFlags >> 3;

    if (state.mode < OPMODE_AHRS_HIGH_GAIN) {
        PeriodicUpdate result;
        result.filter_state = state;
        return result;
    }

    rbs.time = time;
    rbs.orientation = Eigen::Quaterniond(values[0], values[1], values[2], values[3]);
    rbs.orientation = valueNED2NWU(rbs.orientation);
    rbs.angular_velocity = Eigen::Vector3d(values[4], values[5], values[6]) * deg2rad;

    base::Angle latitude, longitude;
    if (state.mode == OPMODE_INS) {
        rbs.velocity = Eigen::Vector3d(values[7], -values[8], -values[9]);

        latitude = base::Angle::fromDeg(lat_lon_alt[0]);
        longitude = base::Angle::fromDeg(lat_lon_alt[1]);
        rbs.position.z() = lat_lon_alt[2];
    }

    MagneticInfo magnetic_info;
    magnetic_info.time = rbs.time;
    magnetic_info.magnetometers = Eigen::Vector3d(magInfo[0], magInfo[1], magInfo[2]);
    magnetic_info.measured_euler_angles =
        Eigen::Vector3d(magInfo[3], magInfo[4], magInfo[5]);
    magnetic_info.declination = base::Angle::fromRad(magInfo[6]);

    PeriodicUpdate result;
    result.latitude = latitude;
    result.longitude = longitude;
    result.rbs = rbs;
    result.rba = rba;
    result.magnetic_info = magnetic_info;
    result.filter_state = state;
    return result;
}

static Eigen::Matrix3d arrayToCovarianceMatrix(std::array<float, 6> const& v) {
    Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
    m(0, 0) = v[0];
    m(0, 1) = v[1];
    m(0, 2) = v[2];
    m(1, 1) = v[3];
    m(1, 2) = v[4];
    m(2, 2) = v[5];
    return m + m.triangularView<Eigen::StrictlyUpper>().transpose().toDenseMatrix();
}

static Eigen::Matrix4d arrayToCovarianceMatrix(std::array<float, 10> const& v) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
    m(0, 0) = v[0];
    m(0, 1) = v[1];
    m(0, 2) = v[2];
    m(0, 3) = v[3];
    m(1, 1) = v[4];
    m(1, 2) = v[5];
    m(1, 3) = v[6];
    m(2, 2) = v[7];
    m(2, 3) = v[8];
    m(3, 3) = v[9];
    return m + m.triangularView<Eigen::StrictlyUpper>().transpose().toDenseMatrix();
}

template<typename ArrayT, size_t Size>
static Eigen::Matrix<double, Size, 1> arrayToVector(std::array<ArrayT, Size> const& a) {
    Eigen::Matrix<ArrayT, Size, 1> m(a.data());
    return m.template cast<double>();
}

PeriodicUpdate protocol::parseE5Output(uint8_t const* buffer, int bufferSize)
{
    uint8_t const* end = buffer + bufferSize;
    uint8_t const* cursor = buffer;

    uint32_t time_ms;
    cursor = endianness::decode<uint32_t>(cursor, time_ms, end);
    uint8_t filterFlags;
    cursor = endianness::decode<uint8_t>(cursor, filterFlags, end);
    int8_t board_temperature_C;
    cursor = endianness::decode<int8_t>(cursor, board_temperature_C, end);

    std::array<float, 4> q;
    cursor = endianness::decode(cursor, q, end);
    std::array<float, 3> angular_velocity;
    cursor = endianness::decode(cursor, angular_velocity, end);
    std::array<float, 3> velocity;
    cursor = endianness::decode(cursor, velocity, end);
    std::array<double, 2> lat_lon;
    cursor = endianness::decode(cursor, lat_lon, end);
    float alt;
    cursor = endianness::decode(cursor, alt, end);

    std::array<float, 3> magnetometers;
    cursor = endianness::decode(cursor, magnetometers, end);
    std::array<float, 3> measured_euler_angles;
    cursor = endianness::decode(cursor, measured_euler_angles, end);
    float declination;
    cursor = endianness::decode(cursor, declination, end);

    std::array<float, 3> accelerations;
    cursor = endianness::decode(cursor, accelerations, end);

    std::array<float, 6> covPosition;
    cursor = endianness::decode(cursor, covPosition, end);
    std::array<float, 6> covVelocity;
    cursor = endianness::decode(cursor, covVelocity, end);
    std::array<float, 10> covQuaternions;
    cursor = endianness::decode(cursor, covQuaternions, end);

    auto time = base::Time::fromMilliseconds(time_ms);

    FilterState state;
    state.time = time;
    state.mode = static_cast<FilterMode>(filterFlags & 0x7);
    state.status = filterFlags >> 3;

    if (state.mode < OPMODE_AHRS_HIGH_GAIN) {
        PeriodicUpdate result;
        result.filter_state = state;
        result.board_temperature = base::Temperature::fromCelsius(board_temperature_C);
        return result;
    }

    base::samples::RigidBodyState rbs;
    rbs.time = time;
    auto q_ned = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
    rbs.orientation = valueNED2NWU(q_ned);
    rbs.angular_velocity = arrayToVector(angular_velocity);

    base::Angle latitude, longitude;
    if (state.mode == OPMODE_INS) {
        // IMU uses NED frame
        Eigen::Vector3d velocity_ned = arrayToVector(velocity);
        rbs.velocity = velocity_ned.cwiseProduct(Eigen::Vector3d(1, -1, -1));

        latitude = base::Angle::fromRad(lat_lon[0]);
        longitude = base::Angle::fromRad(lat_lon[1]);
        rbs.position.z() = alt;
        rbs.cov_position = arrayToCovarianceMatrix(covPosition);
        rbs.cov_velocity = arrayToCovarianceMatrix(covVelocity);
    }

    MagneticInfo magnetic_info;
    magnetic_info.time = time;
    magnetic_info.magnetometers = arrayToVector(magnetometers);
    magnetic_info.measured_euler_angles = arrayToVector(measured_euler_angles);
    magnetic_info.declination = base::Angle::fromRad(declination);

    base::samples::RigidBodyAcceleration rba;
    rba.time = rbs.time;
    rba.acceleration = arrayToVector(accelerations);

    PeriodicUpdate result;
    result.rbs = rbs;
    result.rba = rba;
    result.latitude = latitude;
    result.longitude = longitude;
    result.covQuaternion = arrayToCovarianceMatrix(covQuaternions);
    result.magnetic_info = magnetic_info;
    result.board_temperature = base::Temperature::fromCelsius(board_temperature_C);
    result.filter_state = state;
    return result;
}