cmake_minimum_required(VERSION 2.8.3)
project(laserscan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  laser_geometry
  tf
  sensor_msgs
  message_filters
)

add_message_files(
   FILES
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laserscan
#  CATKIN_DEPENDS roscpp rospy std_msg
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_projector 
  src/scan_projector.cpp
)

target_link_libraries(scan_projector 
  ${catkin_LIBRARIES}
)

