#ifndef IMU_ACEINNA_OPENIMU_CONFIGURATION_HPP
#define IMU_ACEINNA_OPENIMU_CONFIGURATION_HPP

#include <string>

namespace imu_aceinna_openimu {
    enum ORIENTATION_AXIS {
        ORIENTATION_AXIS_PLUS_X,
        ORIENTATION_AXIS_MINUS_X,
        ORIENTATION_AXIS_PLUS_Y,
        ORIENTATION_AXIS_MINUS_Y,
        ORIENTATION_AXIS_PLUS_Z,
        ORIENTATION_AXIS_MINUS_Z
    };

    struct Configuration {
        struct Orientation {
            ORIENTATION_AXIS forward = ORIENTATION_AXIS_PLUS_X;
            ORIENTATION_AXIS right = ORIENTATION_AXIS_PLUS_Y;
            ORIENTATION_AXIS down = ORIENTATION_AXIS_PLUS_Z;

            bool operator ==(Orientation const& other) const;
            bool operator !=(Orientation const& other) const;
        };

        std::string periodic_packet_type = "z1";
        int periodic_packet_rate = 10;
        int acceleration_low_pass_filter = 25;
        int angular_velocity_low_pass_filter = 25;

        Orientation orientation;
    };

    inline std::string to_string(std::string const& value) { return value; }
    template<typename T>
    std::string to_string(T const& value) { return std::to_string(value); }
    std::string to_string(Configuration::Orientation const& orientation);
}

#endif
