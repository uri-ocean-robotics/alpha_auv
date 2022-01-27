cmake_minimum_required(VERSION 3.0.2)
project(ALPHA_COMMON VERSION 1.0.0)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE include)

target_link_libraries(${PROJECT_NAME})
