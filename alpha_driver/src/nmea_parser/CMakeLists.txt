cmake_minimum_required(VERSION 3.0.2)
project(NMEA_PARSER)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

include_directories(include)

add_library(${PROJECT_NAME} src/nmea.cpp)

target_include_directories (${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME})
