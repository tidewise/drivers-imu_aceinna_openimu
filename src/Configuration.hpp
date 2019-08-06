#ifndef IMU_ACEINNA_OPENIMU_CONFIGURATION_HPP
#define IMU_ACEINNA_OPENIMU_CONFIGURATION_HPP

#include <string>

namespace imu_aceinna_openimu {
    struct Configuration {
        std::string periodic_packet_type;
        int periodic_packet_rate;
        int acceleration_low_pass_filter;
        int angular_velocity_low_pass_filter;
        std::string orientation;
    };
}

#endif
