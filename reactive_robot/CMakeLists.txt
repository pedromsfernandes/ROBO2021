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
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
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
)


####################### Wall following ##################################

# For C++ Projects

#add_executable(stdr_wall_following
#  src/wall_following/main.cpp
#  src/wall_following/wall_following.cpp)
#add_dependencies(stdr_wall_following ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
#target_link_libraries(stdr_wall_following
#  ${catkin_LIBRARIES}
#)

# Install excecutables
#install(TARGETS stdr_wall_following
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# For Python projects

catkin_install_python(PROGRAMS src/wall_following/main.py
  DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
)