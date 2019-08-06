#ifndef IMU_ACEINNA_OPENIMU_CONFIGURATION_HPP
#define IMU_ACEINNA_OPENIMU_CONFIGURATION_HPP

#include <string>

namespace imu_aceinna_openimu {
    struct Configuration {
        std::string periodic_packet_type = "z1";
        int periodic_packet_rate = 10;
        int acceleration_low_pass_filter = 25;
        int angular_velocity_low_pass_filter = 25;
        std::string orientation = "+X+Y+Z";
    };
}

#endif
