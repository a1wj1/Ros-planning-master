cmake_minimum_required(VERSION 3.0.2)
project(global_routing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(SRC_FILE
 src/frenet.cpp
 src/global_routing.cpp
 src/minimumjerk.cpp
 src/trajectory_optimization.cpp
 src/global_routing_node.cpp
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  planning_msgs
  geometry_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  lanelet2_extension
  autoware_planning_msgs
  tf2_ros
  tf2_geometry_msgs
)
find_package(Eigen3 QUIET)


catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES lanelet2_extension_lib
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS
   geometry_msgs
   lanelet2_core
   lanelet2_io
   lanelet2_maps
   lanelet2_projection
   lanelet2_routing
   lanelet2_traffic_rules
  DEPENDS
  Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/global_routing
  ${catkin_INCLUDE_DIRS}
)



## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(global_routing ${SRC_FILE})
target_link_libraries(global_routing ${catkin_LIBRARIES})
add_executable(global_routing_node ${SRC_FILE})
target_link_libraries(global_routing_node ${catkin_LIBRARIES})

