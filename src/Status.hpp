#ifndef IMU_ACEINNA_OPENIMU_STATUS_HPP
#define IMU_ACEINNA_OPENIMU_STATUS_HPP

#include <base/Time.hpp>
#include <base/Temperature.hpp>

namespace imu_aceinna_openimu {
    struct Status {
        base::Time time;
        base::Time last_good_gps;
        base::Time last_usable_gps_velocity;
        base::Temperature temperature;
    };
}

#endif