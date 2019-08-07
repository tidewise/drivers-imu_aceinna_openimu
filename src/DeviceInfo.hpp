#ifndef IMU_ACEINNA_OPENIMU_DEVICEINFO_HPP
#define IMU_ACEINNA_OPENIMU_DEVICEINFO_HPP

#include <string>

namespace imu_aceinna_openimu {
    struct DeviceInfo {
        std::string device_id;
        std::string app_version;
    };
}

#endif