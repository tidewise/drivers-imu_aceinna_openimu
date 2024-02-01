#ifndef IMU_ACEINNA_OPENIMU_MAGNETICCALIBRATION_HPP
#define IMU_ACEINNA_OPENIMU_MAGNETICCALIBRATION_HPP

#include <base/Eigen.hpp>
#include <base/Angle.hpp>

namespace imu_aceinna_openimu {
    /** Configuration of magnetic distortion compensation
     *
     * See README for the general procedure
     */
    struct MagneticCalibration {
        base::Vector2d hard_iron = base::Vector2d::Zero();
        base::Angle soft_iron_angle = base::Angle::fromRad(0);
        float soft_iron_ratio = 1;
    };
}

#endif