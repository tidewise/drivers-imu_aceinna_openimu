#ifndef IMU_ACEINNA_OPENIMU_MAGNETICALIGNMENTSTATE_HPP
#define IMU_ACEINNA_OPENIMU_MAGNETICALIGNMENTSTATE_HPP

namespace imu_aceinna_openimu {
    enum MagneticAlignmentState {
        MA_STOPPED = 0,
        MA_RUNNING_WITH_AUTOEND = 1,
        MA_RUNNING = 2
    };
}

#endif