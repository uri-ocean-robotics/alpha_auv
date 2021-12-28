cmake_minimum_required(VERSION 3.0.2)
project(alpha_teleop)


find_package(catkin REQUIRED COMPONENTS
  joy_teleop
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES alpha_teleop
#  CATKIN_DEPENDS joy_teleop
#  DEPENDS system_lib
)
