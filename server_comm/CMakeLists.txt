cmake_minimum_required(VERSION 3.0.2)
project(server_comm)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs rospy mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/server_comm/comm_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
