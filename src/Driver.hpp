#ifndef IMU_ACEINNA_OPENIMU_DRIVER_HPP
#define IMU_ACEINNA_OPENIMU_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include <imu_aceinna_openimu/Configuration.hpp>

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

        std::string mDeviceInfo;
        std::string queryDeviceInfo();

    public:
        Driver();
        void openURI(std::string const& uri);

        std::string getDeviceInfo() const;
        Configuration readConfiguration();

        void setBaudrate(int rate);

        void writeConfiguration(Configuration const& configuration, bool validate = false);

        template<typename T>
        void writeConfiguration(int index, T value, bool validate = false);

        void saveConfiguration();

        template<typename T>
        T readConfiguration(int index);
    };
}

#endif