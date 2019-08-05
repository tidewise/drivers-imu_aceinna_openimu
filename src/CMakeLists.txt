rock_library(imu_aceinna_openimu
    SOURCES Protocol.cpp Driver.cpp
    HEADERS Protocol.hpp Driver.hpp
    DEPS_PKGCONFIG iodrivers_base)

rock_executable(imu_aceinna_openimu_ctl Main.cpp
    DEPS imu_aceinna_openimu)

