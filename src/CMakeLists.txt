rock_library(imu_aceinna_openimu
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(imu_aceinna_openimu_bin Main.cpp
    DEPS imu_aceinna_openimu)

