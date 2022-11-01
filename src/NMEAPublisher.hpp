#ifndef IMU_ACEINNA_OPENIMU_NMEAPUBLISHER_HPP
#define IMU_ACEINNA_OPENIMU_NMEAPUBLISHER_HPP

#include <imu_aceinna_openimu/Driver.hpp>
#include <iodrivers_base/Driver.hpp>

namespace imu_aceinna_openimu {
    class PeriodicUpdate;

    /** A class that "sits" between the IMU and the "real" driver to convert data
     * into NMEA sentences and publish them */
    class NMEAPublisher {
        static constexpr int32_t BUFFER_SIZE = 32768;

        class RawIODriver : public iodrivers_base::Driver {
            int extractPacket(uint8_t const* buffer, size_t buffer_size) const
            {
                return 0;
            }

        public:
            RawIODriver()
                : Driver(BUFFER_SIZE)
            {
            }
        };

        std::vector<uint8_t> m_buffer;

        Driver m_device;
        RawIODriver m_forward;
        base::Time m_forward_read_timeout;
        RawIODriver m_nmea;

        int forwardToDevice();
        std::pair<bool, PeriodicUpdate> forwardFromDevice();

    public:
        NMEAPublisher();

        void openDevice(std::string const& uri);
        void openForward(std::string const& uri, base::Time const& read_timeout);
        void openNMEA(std::string const& uri);

        bool process(base::Time const& timeout);
        void publishNMEA(PeriodicUpdate const& update);

        void configureIMU();
    };
}

#endif
