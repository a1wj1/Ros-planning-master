cmake_minimum_required(VERSION 3.0.2)
project(ui_console)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  planning_msgs
)


find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  src/ui_console.cpp
  src/mainwindow.cpp
)
set(FORMS
  src/mainwindow.ui
)

add_message_files(
  FILES 
  control.msg
)
generate_messages(
  DEPENDENCIES 
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qt_interface_console
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib

#  CATKIN_DEPENDS 
#  roscpp 
#  rospy 
#  std_msgs 
#  message_runtime
)

include_directories(
  include/interface_console
  ${catkin_INCLUDE_DIRS}
)

add_executable(ui_console ${SOURCES} ${FORMS})
target_link_libraries(ui_console
  ${catkin_LIBRARIES}
)
target_link_libraries(ui_console
  Qt5::Widgets
)
