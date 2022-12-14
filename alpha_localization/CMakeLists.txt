cmake_minimum_required(VERSION 3.0.2)
project(alpha_localization)

find_package(catkin REQUIRED COMPONENTS
  robot_localization
  roscpp
  uno_lqe
  std_msgs
  geometry_msgs
  mvp_msgs
  sensor_msgs
  imu_transformer
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES alpha_localization
#  CATKIN_DEPENDS robot_localization
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/alpha_localization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(imu_ned_to_enu src/imu_ned_enu.cpp)
add_executable(depth_filter_node src/depth_filter_node.cpp)
add_executable(pressure_to_depth_node src/pressure_to_depth_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(imu_ned_to_enu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(depth_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pressure_to_depth_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(imu_ned_to_enu
  ${catkin_LIBRARIES}
)

target_link_libraries(depth_filter_node
  ${catkin_LIBRARIES}
)

target_link_libraries(pressure_to_depth_node
  ${catkin_LIBRARIES}
)
