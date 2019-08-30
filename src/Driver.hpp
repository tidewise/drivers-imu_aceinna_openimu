#ifndef IMU_ACEINNA_OPENIMU_DRIVER_HPP
#define IMU_ACEINNA_OPENIMU_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include <imu_aceinna_openimu/DeviceInfo.hpp>
#include <imu_aceinna_openimu/Configuration.hpp>
#include <imu_aceinna_openimu/EKFWithCovariance.hpp>
#include <imu_aceinna_openimu/Status.hpp>
#include <iosfwd>

namespace imu_aceinna_openimu {
    struct ConfigurationWriteFailed : public std::runtime_error {
        using std::runtime_error::runtime_error;
    };

    class Driver : public iodrivers_base::Driver {
    public:
        enum UpdateType
        {
            UPDATED_STATE = 1,
            UPDATED_RAW_IMU_SENSORS = 2,
            UPDATED_STATUS = 4,
            UPDATED_IGNORED
        };

    private:
        static const int BUFFER_SIZE = 256 * 15;
        uint8_t mWriteBuffer[BUFFER_SIZE];
        uint8_t mReadBuffer[BUFFER_SIZE];

        Status mStatus;
        EKFWithCovariance mEKFWithCovariance;

        template<typename T>
        void writeConfigurationGeneric(int index, T value, bool validate);

        virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

        /** Read packets until a packet of type 'command' is found
         */
        int readPacketsUntil(uint8_t* buffer, int bufferSize, uint8_t const* command,
                             base::Time timeout);


        /** @overload */
        int readPacketsUntil(uint8_t* buffer, int bufferSize, uint8_t const* command);

        /** Internal processOne implementation that allows to process single messages
         * as well as EP-multiplexed messages
         */
        UpdateType processOne(uint8_t const* type, uint8_t const* message, uint8_t len);

    public:
        Driver();

        struct UnsupportedDevice : public std::runtime_error {
            using std::runtime_error::runtime_error;
        };

        /** Validate that the device and its firmware are compatible with
         * this driver
         */
        DeviceInfo validateDevice(bool allow_bootloader = false);

        /** Read the device information */
        DeviceInfo readDeviceInfo();

        /** Read the current device configuration */
        Configuration readConfiguration();

        /** Read the current device status */
        Status readStatus();

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

        /** @overload
         */
        void writeExtendedPeriodMessageConfiguration(std::string name, int period);

        /** Configure message periods in extended packet period mode (EP) */
        void writeExtendedPeriodMessageConfiguration(int index, int period);

        /**
         * Configure the cutoff frequency for the acceleration low-pass filter
         * (Hz)
         *
         * @param rate the cutoff frequency. Can be 0, 2, 5, 10, 20, 25
         */
        void writeAccelerationLowPassFilter(int64_t rate);

        /**
         * Configure the cutoff frequency for the angular velocity low-pass
         * filter (Hz)
         *
         * @param rate the cutoff frequency. Can be 0, 2, 5, 10, 20, 25
         */
        void writeAngularVelocityLowPassFilter(int64_t rate);

        /** Configure the GPS protocol */
        void writeGPSProtocol(GPSProtocol protocol);

        /** Configure the GPS baud rate */
        void writeGPSBaudrate(int baudrate);

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

        struct UpdateResult
        {
            int updated = 0;

            void add(UpdateType type);
            bool isUpdated(UpdateType type) const;
        };

        UpdateResult processOne();

        /** Return the last state received by processOne
         */
        EKFWithCovariance getState() const;

        /** Return the last IMU status received by processOne
         *
         * This is named getIMUStatus() to avoid clashing with iodrivers_base::Driver::getStatus()
         */
        Status getIMUStatus() const;

        /** Write a new app firmware */
        void writeFirmware(std::vector<uint8_t> const& data,
                           std::ostream& progress = nullStream());
    };
}

#endif