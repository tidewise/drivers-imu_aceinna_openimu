rock_library(imu_aceinna_openimu
    SOURCES Protocol.cpp
    HEADERS Protocol.hpp)

rock_executable(imu_aceinna_openimu_ctl Main.cpp
    DEPS imu_aceinna_openimu)

