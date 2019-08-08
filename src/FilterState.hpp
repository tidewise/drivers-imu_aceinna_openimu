#ifndef IMU_ACEINNA_OPENIMU_FILTERSTATE_HPP
#define IMU_ACEINNA_OPENIMU_FILTERSTATE_HPP

#include <base/Time.hpp>
#include <cstdint>

namespace imu_aceinna_openimu {
    enum FilterMode {
        OPMODE_STABILIZING,
        OPMODE_INITIALIZING,
        OPMODE_AHRS_HIGH_GAIN,
        OPMODE_AHRS_LOW_GAIN,
        OPMODE_INS
    };

    enum FilterStatus {
        LINEAR_ACCELERATION = 1,
        TURN_SWITCH = 2,
        COURSE_USED_AS_HEADING = 4
    };

    struct FilterState {
        base::Time time;

        FilterMode mode;

        /* @meta bitfield /imu_aceinna_openimu/FilterStatus */
        int32_t status;

        std::string toString() const;
    };
}

#endif