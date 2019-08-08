#ifndef IMU_ACEINNA_OPENIMU_PROTOCOL_HPP
#define IMU_ACEINNA_OPENIMU_PROTOCOL_HPP

#include <cstdint>
#include <string>
#include <imu_aceinna_openimu/Configuration.hpp>
#include <imu_aceinna_openimu/EKFWithCovariance.hpp>

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
        // Biggest block that can be written when flashing the firmware
        static const int MAX_APP_BLOCK_SIZE = 240;

        enum WriteStatus {
            WRITE_STATUS_OK = 0,
            WRITE_STATUS_INVALID_INDEX,
            WRITE_STATUS_INVALID_VALUE,
            WRITE_STATUS_UNKNOWN
        };

        /** Decode the +X-Y+Z format from the configuration message into
         *  Configuration::Orientation */
        Configuration::Orientation decodeOrientationString(std::string axis);

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

        /** Writes a device info query (pG) to the given buffer
         */
        uint8_t* queryDeviceID(uint8_t* buffer);

        /** Parse a device info response
         */
        std::string parseDeviceID(uint8_t const* payload, int size);

        /** Writes a app version query (gV) to the given buffer
         */
        uint8_t* queryAppVersion(uint8_t* buffer);

        /** Parse a device info response
         */
        std::string parseAppVersion(uint8_t const* payload, int size);

        /** Query the configuration parameters (gA) */
        uint8_t* queryConfiguration(uint8_t* buffer);

        /** Parse configuration parameters message (gA) */
        Configuration parseConfiguration(uint8_t const* buffer, int bufferSize);

        /** Write a int64 parameter (uP)
         *
         * T can only be int64_t and std::string. Any other type will fail at
         * linking time
         */
        template<typename T>
        uint8_t* writeConfiguration(uint8_t* buffer, int index, T value);

        /** Write an orientation configuration parameters */
        uint8_t* writeConfiguration(uint8_t* buffer, int index,
                                    Configuration::Orientation axes);

        /** Parse the status from the configuration write reply */
        WriteStatus parseWriteConfigurationStatus(uint8_t* buffer, int bufferSize);

        /** Parse configuration parameters message (gA) */
        Configuration parseConfiguration(uint8_t const* buffer, int bufferSize);

        /** Query the value of a single configuration parameter (gP) */
        uint8_t* queryConfigurationParameter(uint8_t* buffer, int index);

        /** Parse a read of a single configuration property (gP)
         *
         * T can only be int64_t and std::string. Any other type will fail at
         * linking time
         */
        template<typename T>
        T parseConfigurationParameter(uint8_t* buffer, int bufferSize, int expectedIndex);

        /** Restore default configuration and save it to flash
         */
        uint8_t* queryRestoreDefaultConfiguration(uint8_t* buffer);

        /** Reset the unit
         */
        uint8_t* queryReset(uint8_t* buffer);

        /** Save current configuration to flash
         */
        uint8_t* queryConfigurationSave(uint8_t* buffer);

        /** Make the unit switch to bootloader mode (JI)
         */
        uint8_t* queryJumpToBootloader(uint8_t* buffer);

        /** Make the unit switch to app mode (JA)
         */
        uint8_t* queryJumpToApp(uint8_t* buffer);

        /** Write an app block
         */
        uint8_t* queryAppBlockWrite(uint8_t* buffer, uint32_t address,
                                    uint8_t const* blockData, int blockSize);


        /** Parse the EKF-with-covariance response (e3) */
        EKFWithCovariance parseEKFWithCovariance(uint8_t const* buffer, int bufferSize);
    }
}

#endif