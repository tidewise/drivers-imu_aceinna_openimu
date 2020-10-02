#ifndef IMU_ACEINNA_OPENIMU_MAGNETICINFO_HPP
#define IMU_ACEINNA_OPENIMU_MAGNETICINFO_HPP

#include <base/Time.hpp>
#include <base/Angle.hpp>
#include <base/Eigen.hpp>

namespace imu_aceinna_openimu {
    /**
     * Ad-hoc structure to hold information about yaw resolution using magnetic
     * sensors
     */
    struct MagneticInfo {
        base::Time time;
        base::Angle declination;
        base::Vector3d magnetometers;
        base::Vector3d measured_euler_angles;
    };
}

#endif
