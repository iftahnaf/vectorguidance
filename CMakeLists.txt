cmake_minimum_required(VERSION 3.10)
project(main)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find the Eigen3 and Google Test packages
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

# Add a library for vector guidance
add_library(vectorguidance STATIC 
            src/common.cpp
            src/bounded_interception.cpp
            src/soft_landing.cpp)
target_include_directories(vectorguidance PUBLIC ${EIGEN3_INCLUDE_DIR})
set_target_properties(vectorguidance PROPERTIES POSITION_INDEPENDENT_CODE ON)


# Specify the installation directory for the library
install(TARGETS vectorguidance
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Specify the installation directory for the header files
install(DIRECTORY include/
    DESTINATION include/vectorguidance
    FILES_MATCHING PATTERN "*.hpp"
)

# Add the soft_landing executable
option(BUILD_EXAMPLES "Build examples" ON)
if (BUILD_EXAMPLES)
    add_executable(example_soft_landing examples/example_soft_landing.cpp)
    target_include_directories(example_soft_landing PUBLIC ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(example_soft_landing PUBLIC vectorguidance)

    add_executable(example_soft_landing_bounded examples/example_soft_landing_bounded.cpp)
    target_include_directories(example_soft_landing_bounded PUBLIC ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(example_soft_landing_bounded PUBLIC vectorguidance)

    add_executable(example_bounded_interception examples/example_bounded_interception.cpp)
    target_include_directories(example_bounded_interception PUBLIC ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(example_bounded_interception PUBLIC vectorguidance)
endif()

# Conditionally add the test executable based on a flag
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    add_executable(test_soft_landing tests/test_soft_landing.cpp)
    target_include_directories(test_soft_landing PUBLIC ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(test_soft_landing PUBLIC vectorguidance GTest::GTest GTest::Main)

    add_executable(test_bounded_interception tests/test_bounded_interception.cpp)
    target_include_directories(test_bounded_interception PUBLIC ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(test_bounded_interception PUBLIC vectorguidance GTest::GTest GTest::Main)
endif()
