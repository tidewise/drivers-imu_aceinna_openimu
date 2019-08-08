#ifndef IMU_ACEINNA_OPENIMU_DRIVER_HPP
#define IMU_ACEINNA_OPENIMU_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include <imu_aceinna_openimu/DeviceInfo.hpp>
#include <imu_aceinna_openimu/Configuration.hpp>
#include <imu_aceinna_openimu/EKFWithCovariance.hpp>
#include <iosfwd>

namespace imu_aceinna_openimu {
    struct ConfigurationWriteFailed : public std::runtime_error {
        using std::runtime_error::runtime_error;
    };

    class Driver : public iodrivers_base::Driver {
    private:
        static const int BUFFER_SIZE = 256 * 15;
        uint8_t mWriteBuffer[BUFFER_SIZE];
        uint8_t mReadBuffer[BUFFER_SIZE];

        template<typename T>
        void writeConfigurationGeneric(int index, T value, bool validate);

        virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

        /** Read packets until a packet of type 'command' is found
         */
        int readPacketsUntil(uint8_t* buffer, int bufferSize, uint8_t const* command,
                             base::Time timeout);


        /** @overload */
        int readPacketsUntil(uint8_t* buffer, int bufferSize, uint8_t const* command);

        DeviceInfo mDeviceInfo;
        DeviceInfo readDeviceInfo();

    public:
        Driver();

        struct UnsupportedDevice : public std::runtime_error {
            using std::runtime_error::runtime_error;
        };

        /**
         * @param validateDevice validates that the openIMU device firmware is
         *                       compatible with this driver. See README.md.
         */
        void openURI(std::string const& uri, bool validateDevice = true);

        /** Information about the device */
        DeviceInfo getDeviceInfo() const;

        /** Whether the device is in bootloader mode or in app mode */
        bool isBootloaderMode() const;

        /** Read the current device configuration */
        Configuration readConfiguration();

        /** Change the UART baud rate
         *
         * The change will be effective only if (1) the configuration is saved
         * to flash and (2) the unit is reset.
         */
        void writeBaudrate(int rate);

        /** Reset the unit */
        void queryReset();

        /** Restore the default configuration and save it to flash */
        void queryRestoreDefaultConfiguration();

        /** Write a whole configuration */
        void writeConfiguration(Configuration const& configuration, bool validate = false);

        /** Write a single configuration parameter
         *
         * See Protocol.md for the various fields and their possible values.
         * The type must either be int64_t or std::string. Other values will fail
         * at link time
         *
         * @param validate if true, the method will re-read the configuration parameter
         *        to ensure it has been properly updated
         */
        template<typename T>
        void writeConfiguration(int index, T value, bool validate = false);

        /** Configure the periodic packet */
        void writePeriodicPacketConfiguration(std::string packet, int rate);

        /** Configure which sensors can be used by the algorithm */
        void writeUsedSensors(bool magnetometers, bool gps, bool gps_course_as_heading);

        /** Save the configuration to flash */
        void saveConfiguration();

        /** Read a single configuration parameter
         *
         * See Protocol.md for the various fields and their possible values.
         * The type must either be int64_t or std::string. Other values will fail
         * at link time.
         */
        template<typename T>
        T readConfiguration(int index);

        /** Switch to bootloader mode
         *
         * You need to reconnect after this. To force the issue, the method
         * calls close(), which essentially invalidates the driver
         */
        void toBootloader();

        /** Switch to app mode
         *
         * You need to reconnect after this. To force the issue, the method
         * calls close(), which essentially invalidates the driver
         */
        void toApp();

        static std::ostream& nullStream();

        EKFWithCovariance pollEKFWithCovariance();

        /** Write a new app firmware */
        void writeFirmware(std::vector<uint8_t> const& data,
                           std::ostream& progress = nullStream());
    };
}

#endif