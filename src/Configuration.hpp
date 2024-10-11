#ifndef IMU_ACEINNA_OPENIMU_CONFIGURATION_HPP
#define IMU_ACEINNA_OPENIMU_CONFIGURATION_HPP

#include <base/Angle.hpp>
#include <base/Eigen.hpp>
#include <string>

namespace imu_aceinna_openimu {
    enum GPSProtocol {
        GPS_AUTO = -1,
        GPS_UBLOX = 0,
        GPS_NOVATEL_BINARY = 1,
        GPS_NOVATEL_ASCII = 2,
        GPS_NMEA0183 = 3,
        GPS_SIRF_BINARY = 4,
        GPS_LAST_KNOWN_PROTOCOL = GPS_SIRF_BINARY
    };

    struct Configuration {
        std::string periodic_packet_type = "z1";
        int16_t periodic_packet_rate = 10;
        int16_t acceleration_low_pass_filter = 25;
        int16_t angular_velocity_low_pass_filter = 25;

        /** Axis mapping between the physical IMU frame and the IMU output frame
         *
         * The three slots represent the X, Y and Z output axis of the IMU. The
         * value in the slot is the IMU physical axis on which the output axis
         * will be mapped.
         *
         * For instance, +Y+X-Z means that the X of the IMU's output frame is
         * actually the +Y axis of the physical IMU, the Y of the IMU output
         * frame is the +X of the physical IMU and Z is swapped
         *
         * Because of how the IMU filter is implemented, for best performance,
         * the mapping must always keep the right-handed rule, should ensure
         * that the (X, Y) plane is horizontal in the general case and that Z is
         * pointing down.
         *
         * You can validate the last two criteria by looking at the measured
         * roll and pitch columns of imu_aceinna_openimu_ctl poll-mag, and
         * making sure they are zero. Do **not** use poll-pose for this as
         * poll-pose converts to the NWU convention, which is *not* the IMU's
         * internal convention
         */
        std::string orientation = "+X+Y+Z";

        GPSProtocol gps_protocol = GPS_UBLOX;
        int32_t gps_baud_rate = 115200;

        double hard_iron[2] = { 0, 0 };
        double soft_iron_ratio = 1;
        double soft_iron_angle = 0;

        base::Vector3d lever_arm = base::Vector3d::Zero();;
        base::Vector3d point_of_interest = base::Vector3d::Zero();

        base::Angle rtk_heading2mag_heading = base::Angle::unknown();

        bool operator ==(Configuration const& other) const;
        bool operator !=(Configuration const& other) const;

        /** Whether the IMU needs to be reset because of changes from original to self */
        bool needsReset(Configuration const& original) const;
    };

    inline std::string to_string(std::string const& value)
    {
        return value;
    }
    template <typename T> std::string to_string(T const& value)
    {
        return std::to_string(value);
    }
}

#endif
