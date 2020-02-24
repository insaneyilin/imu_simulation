cmake_minimum_required(VERSION 3.0)

project(imu_simulation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(. _SRCS)
add_executable(gen_imu_data ${_SRCS})
