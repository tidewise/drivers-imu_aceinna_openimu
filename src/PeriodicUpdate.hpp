#ifndef IMU_ACEINNA_OPENIMU_PERIODICUPDATE_HPP
#define IMU_ACEINNA_OPENIMU_PERIODICUPDATE_HPP

#include <base/Float.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/RigidBodyAcceleration.hpp>
#include <gps_base/UTMConverter.hpp>
#include <base/Angle.hpp>
#include <imu_aceinna_openimu/FilterState.hpp>
#include <imu_aceinna_openimu/MagneticInfo.hpp>

namespace imu_aceinna_openimu {
    /** Data structure that holds the periodic update information
     * from the messages we support (stock e2 and the tidewise-specific e4)
     */
    struct PeriodicUpdate {
        MagneticInfo magnetic_info;

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