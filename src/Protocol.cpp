#include <imu_aceinna_openimu/Protocol.hpp>
#include <imu_aceinna_openimu/Endianness.hpp>
#include <cstring>
#include <stdexcept>
#include <iostream>

using namespace std;
using namespace imu_aceinna_openimu;
using endianness::decode;

struct ConfigurationParameter
{
    int index;
    int offset;
    enum DATA_TYPE { UINT64, INT64, CHAR8 };
    DATA_TYPE type;
    char const* name;
};

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

static const ConfigurationParameter CONFIGURATION[] = {
    { 2, 16, ConfigurationParameter::UINT64, "Baud Rate" },
    { 3, 24, ConfigurationParameter::CHAR8, "Periodic Packet Type" },
    { 3, 32, ConfigurationParameter::INT64, "Periodic Packet Rate" },
    { 3, 40, ConfigurationParameter::INT64, "Acceleration low-pass filter" },
    { 3, 48, ConfigurationParameter::INT64, "Angular velocity low-pass filter" },
    { 3, 56, ConfigurationParameter::CHAR8, "Orientation" }
};

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
    cursor = decode(buffer + 32, ret.periodic_packet_rate, end);
    cursor = decode(cursor, ret.acceleration_low_pass_filter, end);
    cursor = decode(cursor, ret.angular_velocity_low_pass_filter, end);

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
    float magneticCalibration[4];
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
