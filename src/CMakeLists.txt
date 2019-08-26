rock_library(imu_aceinna_openimu
    SOURCES Protocol.cpp Driver.cpp FilterState.cpp Configuration.cpp
            EKFWithCovariance.cpp
    HEADERS Protocol.hpp Driver.hpp Configuration.hpp DeviceInfo.hpp
            Endianness.hpp FilterState.hpp Status.hpp EKFWithCovariance.hpp
    DEPS_PKGCONFIG base-types iodrivers_base gps_base)

rock_executable(imu_aceinna_openimu_ctl Main.cpp
    DEPS imu_aceinna_openimu)

