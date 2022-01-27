cmake_minimum_required(VERSION 3.0.2)
project(alpha_driver_package)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
add_subdirectory(../alpha_driver driver)
add_subdirectory(../alpha_mcu mcu)