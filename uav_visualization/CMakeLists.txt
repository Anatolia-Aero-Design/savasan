cmake_minimum_required(VERSION 2.8.3)
project(uav_visualization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/uav_visualization/iha_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
