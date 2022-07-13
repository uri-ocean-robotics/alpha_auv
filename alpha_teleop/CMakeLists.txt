cmake_minimum_required(VERSION 3.0.2)
project(alpha_teleop)


find_package(catkin REQUIRED COMPONENTS
  joy_teleop
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS joy_teleop
)

