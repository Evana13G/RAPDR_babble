cmake_minimum_required(VERSION 2.8.3)
project(environment)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_ros
  roscpp
  rospack
  rospy
  std_msgs
  message_generation
  geometry_msgs
  genmsg
  agent
)

add_message_files(
FILES
  At.msg
  State.msg
  Predicate.msg
  PredicateList.msg
)

add_service_files(
  FILES 
  HandleEnvironmentSrv.srv 
  ScenarioDataSrv.srv 
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs geometry_msgs gazebo_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

