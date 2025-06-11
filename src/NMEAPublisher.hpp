#ifndef IMU_ACEINNA_OPENIMU_NMEAPUBLISHER_HPP
#define IMU_ACEINNA_OPENIMU_NMEAPUBLISHER_HPP

#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/NMEAMessages.hpp>
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
        std::string m_talker;

        uint32_t m_messages = NMEA_PUBLISH_HDT;

        int forwardToDevice();
        std::pair<bool, PeriodicUpdate> forwardFromDevice();

        /** Adds generic NMEA "envelope" to a message content */
        std::string makeNMEASentence(std::string const& content);

    public:
        NMEAPublisher(std::string const& talker = "GN");

        void selectNMEAMessages(uint32_t messages);

        void openDevice(std::string const& uri);
        void openForward(std::string const& uri, base::Time const& read_timeout);
        void openNMEA(std::string const& uri);

        bool process(base::Time const& timeout);
        void publishNMEA(PeriodicUpdate const& update);

        void configureIMU();

        /** Build a HDT sentence from an IMU update */
        std::string getHDTSentence(base::samples::RigidBodyState const& rbs);

        /** Build a ZDA sentence from Time::now */
        std::string getZDASentence(base::Time const& time = base::Time::now());
    };
}

#endif
