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
        std::cout << "CRC: " << hex << expectedCRC << std::endl;
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
    if (bufferSize < 56) {
        throw std::invalid_argument("buffer size for message gA (configuration) "\
                                    "smaller than 56 bytes");
    }
    Configuration ret;
    ret.periodic_packet_type = string(
        reinterpret_cast<char const*>(buffer + 24),
        reinterpret_cast<char const*>(buffer + 26)
    );
    decode(buffer + 32, ret.periodic_packet_rate);
    decode(buffer + 40, ret.acceleration_low_pass_filter);
    decode(buffer + 48, ret.angular_velocity_low_pass_filter);
    ret.orientation = string(
        reinterpret_cast<char const*>(buffer + 56),
        reinterpret_cast<char const*>(buffer + std::min(64, bufferSize))
    );
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

uint8_t* protocol::queryConfigurationSave(uint8_t* buffer) {
    return formatPacket(buffer, "sC", nullptr, 0);
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
