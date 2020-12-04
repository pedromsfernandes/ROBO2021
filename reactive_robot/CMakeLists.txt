cmake_minimum_required(VERSION 3.0.2)
project(reactive_robot)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  tf
  stdr_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  rospy
  message_generation
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  DistToWall.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    roslib
    roscpp
    tf
    stdr_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
    message_runtime
)

####################### Wall following ##################################

catkin_install_python(PROGRAMS src/wall_following/main.py
  DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
)