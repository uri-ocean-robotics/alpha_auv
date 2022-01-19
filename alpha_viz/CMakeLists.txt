cmake_minimum_required(VERSION 3.0.2)
project(alpha_viz)

find_package(catkin REQUIRED COMPONENTS
  rviz
  rviz_imu_plugin
  rviz_visual_tools
)

catkin_package()

