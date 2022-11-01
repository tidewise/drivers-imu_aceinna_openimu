#ifndef IMU_ACEINNA_OPENIMU_CONFIGURATION_HPP
#define IMU_ACEINNA_OPENIMU_CONFIGURATION_HPP

#include <base/Eigen.hpp>
#include <string>

namespace imu_aceinna_openimu {
    enum OrientationAxis {
        ORIENTATION_AXIS_PLUS_X,
        ORIENTATION_AXIS_MINUS_X,
        ORIENTATION_AXIS_PLUS_Y,
        ORIENTATION_AXIS_MINUS_Y,
        ORIENTATION_AXIS_PLUS_Z,
        ORIENTATION_AXIS_MINUS_Z
    };

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
        struct Orientation {
            OrientationAxis forward = ORIENTATION_AXIS_PLUS_X;
            OrientationAxis right = ORIENTATION_AXIS_PLUS_Y;
            OrientationAxis down = ORIENTATION_AXIS_PLUS_Z;

            Orientation();
            Orientation(OrientationAxis forward,
                OrientationAxis right,
                OrientationAxis down);

            bool operator==(Orientation const& other) const;
            bool operator!=(Orientation const& other) const;
        };

        std::string periodic_packet_type = "z1";
        int16_t periodic_packet_rate = 10;
        int16_t acceleration_low_pass_filter = 25;
        int16_t angular_velocity_low_pass_filter = 25;

        Orientation orientation;

        GPSProtocol gps_protocol = GPS_UBLOX;
        int32_t gps_baud_rate = 115200;

        double hard_iron[2];
        double soft_iron_ratio;
        double soft_iron_angle;

        base::Vector3d lever_arm;
        base::Vector3d point_of_interest;
    };

    inline std::string to_string(std::string const& value)
    {
        return value;
    }
    template <typename T> std::string to_string(T const& value)
    {
        return std::to_string(value);
    }
    std::string to_string(Configuration::Orientation const& orientation);
}

#endif
