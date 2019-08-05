#include <imu_aceinna_openimu/Protocol.hpp>
#include <imu_aceinna_openimu/Endianness.hpp>
#include <cstring>

using namespace std;
using namespace imu_aceinna_openimu;

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

uint8_t* protocol::queryDeviceInfo(uint8_t* buffer)
{
    return protocol::formatPacket(buffer, "pG", nullptr, 0);
}

std::string protocol::parseDeviceInfo(uint8_t const* payload, int size)
{
    return std::string(payload, payload + size);
}
