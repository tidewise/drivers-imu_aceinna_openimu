#ifndef IMU_ACEINNA_OPENIMU_EKFWITHCOVARIANCE_HPP
#define IMU_ACEINNA_OPENIMU_EKFWITHCOVARIANCE_HPP

#include <base/Float.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/RigidBodyAcceleration.hpp>
#include <gps_base/UTMConverter.hpp>
#include <base/Angle.hpp>
#include <imu_aceinna_openimu/FilterState.hpp>

namespace imu_aceinna_openimu {
    /** The firmware's EKF-with-covariance message (e3) */
    struct EKFWithCovariance {
        /** Pose solution
         *
         * Note that the position field is not set. Use e.g. gps_base::UTMConverter
         * to convert the latitude/longitude to position
         *
         * Fields that are set are:
         * - position, velocity and angular velocity covariances
         * - velocity and angular velocity
         */
        base::samples::RigidBodyState rbs;

        /** Acceleration solution
         *
         * Covariances are not set
         */
        base::samples::RigidBodyAcceleration rba;

        FilterState filter_state;

        base::Angle latitude;
        base::Angle longitude;

        void computeNWUPosition(gps_base::UTMConverter const& converter);
    };
}

#endif