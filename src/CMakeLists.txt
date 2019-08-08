rock_library(imu_aceinna_openimu
    SOURCES Protocol.cpp Driver.cpp FilterState.cpp Configuration.cpp
    HEADERS Protocol.hpp Driver.hpp Configuration.hpp DeviceInfo.hpp
            Endianness.hpp FilterState.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)

rock_executable(imu_aceinna_openimu_ctl Main.cpp
    DEPS imu_aceinna_openimu)

