cmake_minimum_required(VERSION 3.0.2)
project(yolov8)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  sensor_msgs
  cv_bridge
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS std_msgs rospy sensor_msgs cv_bridge std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/yolov8/yolov8_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
