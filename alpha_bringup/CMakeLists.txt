cmake_minimum_required(VERSION 3.0.2)
project(alpha_bringup)

find_package(catkin REQUIRED COMPONENTS
  alpha_description
  alpha_driver
  alpha_msgs
  alpha_teleop
  waterlinked_dvl
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES alpha_bringup
#  CATKIN_DEPENDS alpha_description alpha_driver alpha_msgs alpha_teleop waterlinked_a50_ros_driver
#  DEPENDS system_lib
)
