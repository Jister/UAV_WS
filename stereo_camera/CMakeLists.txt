cmake_minimum_required(VERSION 2.8.3)
project(stereo_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_transformer src/odom_transformer.cpp)

target_link_libraries(odom_transformer
  ${catkin_LIBRARIES}
)