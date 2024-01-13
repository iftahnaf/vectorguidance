cmake_minimum_required(VERSION 3.10)
project(main)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find the Eigen3 and Google Test packages
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

# Add the main executable
add_executable(main src/main.cpp src/common.cpp)
target_include_directories(main PUBLIC ${EIGEN3_INCLUDE_DIR})

# Add the soft_landing executable
add_executable(soft_landing src/soft_landing.cpp src/common.cpp)
target_include_directories(soft_landing PUBLIC ${EIGEN3_INCLUDE_DIR})

# Add the test executable

# setting USE_TEST to false
set(USE_TEST TRUE)
add_executable(test_soft_landing tests/test_soft_landing.cpp src/common.cpp src/soft_landing.cpp)
target_include_directories(test_soft_landing PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(test_soft_landing GTest::GTest GTest::Main)