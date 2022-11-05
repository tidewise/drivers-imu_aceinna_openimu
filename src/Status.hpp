#ifndef IMU_ACEINNA_OPENIMU_STATUS_HPP
#define IMU_ACEINNA_OPENIMU_STATUS_HPP

#include <base/Temperature.hpp>
#include <base/Time.hpp>
#include <imu_aceinna_openimu/FilterState.hpp>

namespace imu_aceinna_openimu {
    struct Status {
        base::Time time;
        uint32_t extended_periodic_packets_overflow = 0;
        uint32_t gps_updates = 0;
        uint32_t gps_rx = 0;
        uint32_t gps_overflows = 0;
        base::Time last_gps_message;
        base::Time last_good_gps;
        base::Time last_usable_gps_velocity;
        base::Temperature temperature;
        float hdop = 50;

        FilterState filter_state;
    };
}

#endif