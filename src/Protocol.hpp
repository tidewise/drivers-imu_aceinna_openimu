#ifndef IMU_ACEINNA_OPENIMU_PROTOCOL_HPP
#define IMU_ACEINNA_OPENIMU_PROTOCOL_HPP

#include <cstdint>
#include <string>
#include <imu_aceinna_openimu/Configuration.hpp>

namespace imu_aceinna_openimu {
    namespace protocol {
        // Packet start marker. See Protocol.md for specification
        static const uint8_t PACKET_START_MARKER = 0x55;
        // Number of bytes that are not payload bytes in a packet
        static const int PACKET_OVERHEAD = 7;
        // Number of header bytes between start of packet and start of payload
        static const int PAYLOAD_OFFSET = 5;
        // Minimum number of bytes per packet
        static const int MIN_PACKET_SIZE = PACKET_OVERHEAD;
        // Maximum number of bytes per packet
        static const int MAX_PACKET_SIZE = MIN_PACKET_SIZE + 256;

        /** Implements iodrivers_base's extractPacket protocol
         *
         * See iodrivers_base::extractPacket for detailed information
         */
        int extractPacket(const uint8_t* buffer, int bufferSize);

        /** Computes the OpenIMU CRC value for the given byte range
         */
        uint16_t crc(const uint8_t* start, const uint8_t* end);

        /** Generic packet formatting
         */
        uint8_t* formatPacket(uint8_t* buffer, char const* code,
                              uint8_t const* payload, int size);

        /** Writes a device info query to the given buffer
         */
        uint8_t* queryDeviceInfo(uint8_t* buffer);

        /** Parse a device info response
         */
        std::string parseDeviceInfo(uint8_t const* payload, int size);

        /** Query the configuration parameters (gA) */
        uint8_t* queryConfiguration(uint8_t* buffer);

        /** Parse configuration parameters message (gA) */
        Configuration parseConfiguration(uint8_t const* buffer, int bufferSize);
    }
}

#endif