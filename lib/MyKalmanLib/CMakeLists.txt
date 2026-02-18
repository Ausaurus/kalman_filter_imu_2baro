cmake_minimum_required(VERSION 3.14)
project(MyKalmanLib VERSION 1.0.0 LANGUAGES CXX)

# Standard for modern embedded C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen (Required dependency)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Define the library as INTERFACE (Header-only)
add_library(MyKalmanLib INTERFACE)

# Link Eigen so anyone using your lib gets Eigen automatically
target_link_libraries(MyKalmanLib INTERFACE Eigen3::Eigen)

# Expose the include directory
target_include_directories(MyKalmanLib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Only build tests if this is the main project (saves time when including in other projects)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    enable_testing()
    add_subdirectory(tests)
endif()
