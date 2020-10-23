cmake_minimum_required(VERSION 2.8.3)
project(action_primitive_variation)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg message_generation)

## Declare ROS messages and services
add_message_files(FILES)

add_service_files(
FILES
APVSrv.srv
RecordLimbDataSrv.srv
AddActionBreakptSrv.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
