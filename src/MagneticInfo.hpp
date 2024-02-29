#ifndef IMU_ACEINNA_OPENIMU_MAGNETICINFO_HPP
#define IMU_ACEINNA_OPENIMU_MAGNETICINFO_HPP

#include <base/Angle.hpp>
#include <base/Eigen.hpp>
#include <base/Time.hpp>

namespace imu_aceinna_openimu {
    /**
     * Ad-hoc structure to hold information about yaw resolution using magnetic
     * sensors
     */
    struct MagneticInfo {
        base::Time time;

        /** Magnetic declination at the current position
         *
         * This is the value calculated by the IMU using its internal model.
         * The value is ned_mag2ned, where ned_mag is the NED frame aligned with
         * the local magnetic north and ned is the NED frame aligned with the
         * geographic (WGS84) north
         */
        base::Angle declination;

        /** Magnetometers measurements
         *
         * Magnetic disturbance calibration already applied
         */
        base::Vector3d magnetometers;

        /** Euler angle measurement from accelerometers and magnetometers
         *
         * They are the unfiltered measurements. The pitch and roll are measured
         * from the direction of gravity (from the accelerometers) and the yaw
         * from the magnetometers. Declination is already applied if in INS mode.
         */
        base::Vector3d measured_euler_angles;
    };
}

#endif
