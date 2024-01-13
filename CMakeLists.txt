cmake_minimum_required(VERSION 3.10)
project(main)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

add_executable(main src/main.cpp src/common.cpp)
target_include_directories(main PUBLIC ${EIGEN3_INCLUDE_DIR})