#ifndef IMU_ACEINNA_OPENIMU_PROTOCOL_HPP
#define IMU_ACEINNA_OPENIMU_PROTOCOL_HPP

#include <cstdint>

namespace imu_aceinna_openimu {
    namespace protocol {
        // TODO
        static const uint8_t PACKET_START_MARKER = 0x55;
        static const uint8_t MIN_PACKET_SIZE = 7;

        // General packet format
        //
        // [0] 0x55
        // [1] 0x55
        // [2] Packet Type byte 1
        // [3] Packet Type byte 2
        // [4] Payload size
        // ...
        // [N - 1] CRC byte 1
        // [N] CRC byte 2
        //
        // CRC does not include the two start bytes and does include the length field
        // Payload size only includes data between 5 and N-1

        int extractPacket(const uint8_t* buffer, int bufferSize);

        uint16_t crc(const uint8_t* start, const uint8_t* end);
    };
}

#endif