cmake_minimum_required(VERSION 3.0.2)
project(server_comm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  mavros_msgs
  message_generation
)

# Add message files
add_message_files(
  FILES
  KonumBilgisi.msg
  KonumBilgileri.msg
  ServerTime.msg
)

# Add service files if any
# add_service_files(
#   FILES
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs nav_msgs geometry_msgs mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add your executables and link libraries
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
