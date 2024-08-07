cmake_minimum_required(VERSION 3.0.2)
project(gps_navigator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  mavros_msgs
  std_srvs
)


catkin_package(
  CATKIN_DEPENDS rospy std_msgs mavros_msgs std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/gps_navigator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
