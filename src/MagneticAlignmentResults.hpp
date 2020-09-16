#ifndef IMU_ACEINNA_OPENIMU_MAGNETICALIGNMENTRESULTS_HPP
#define IMU_ACEINNA_OPENIMU_MAGNETICALIGNMENTRESULTS_HPP

namespace imu_aceinna_openimu {
    /** Structure containing the result of a magnetic alignment run */
    struct MagneticAlignmentResults {
        struct Data {
            float hardIronBias[2];
            float softIronScaleRatio;
            float softIronAngle;
        };

        Data current;
        Data estimated;
    };
}

#endif