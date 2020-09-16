# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.9)
project(imu_aceinna_openimu
        VERSION 0.1
        DESCRIPTION "Rock driver for AceINNA OpenIMU INS application")
find_package(Rock)
rock_init()

set(CMAKE_CXX_STANDARD 14)
rock_standard_layout()
