# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.0)
find_package(Rock)
rock_init(imu_aceinna_openimu 0.1)

set(CMAKE_CXX_STANDARD 11)
rock_standard_layout()
