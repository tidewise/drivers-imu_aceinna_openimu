#ifndef IMU_ACEINNA_OPENIMU_EKFWITHCOVARIANCE_HPP
#define IMU_ACEINNA_OPENIMU_EKFWITHCOVARIANCE_HPP

#include <base/Float.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/RigidBodyAcceleration.hpp>
#include <base/Angle.hpp>
#include <imu_aceinna_openimu/FilterState.hpp>

namespace imu_aceinna_openimu {
    /** The firmware's EKF-with-covariance message (e3) */
    struct EKFWithCovariance {
        base::samples::RigidBodyState rbs;
        base::samples::RigidBodyAcceleration rba;
        FilterState filter_state;

        base::Angle latitude;
        base::Angle longitude;
    };
}

#endif