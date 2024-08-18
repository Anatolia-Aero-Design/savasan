cmake_minimum_required(VERSION 3.0.2)
project(image_processor)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  sensor_msgs
  cv_bridge
  roscpp 
  message_generation
  server_comm
)

add_message_files(
  FILES
  Yolo_xywh.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs rospy sensor_msgs cv_bridge message_runtime server_comm
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/image_processor/image_processor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
