rock_library(imu_aceinna_openimu
    SOURCES Parameter.cpp Protocol.cpp Driver.cpp FilterState.cpp Configuration.cpp
            PeriodicUpdate.cpp
    HEADERS Parameter.hpp Protocol.hpp Driver.hpp Configuration.hpp DeviceInfo.hpp
            Endianness.hpp FilterState.hpp Status.hpp MagneticInfo.hpp PeriodicUpdate.hpp
            MagneticCalibration.hpp
    DEPS_PKGCONFIG base-types iodrivers_base gps_base)

rock_executable(imu_aceinna_openimu_ctl Main.cpp
    DEPS imu_aceinna_openimu)

rock_executable(imu_aceinna_openimu_nmea_publisher NMEAPublisher.cpp MainNMEAPublisher.cpp
    DEPS imu_aceinna_openimu)
