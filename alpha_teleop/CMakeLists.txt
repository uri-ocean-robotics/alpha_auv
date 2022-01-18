cmake_minimum_required(VERSION 3.0.2)
project(alpha_teleop)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  joy_teleop
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES alpha_teleop
#  CATKIN_DEPENDS joy_teleop
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_teleop_node
  src/keyboard_teleop_node.cpp
  src/keyboard_teleop/keyboard_teleop.cpp
)

#add_dependencies(keyboard_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(keyboard_teleop_node
  ${catkin_LIBRARIES}
  ncurses
)



