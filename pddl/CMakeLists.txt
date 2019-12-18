cmake_minimum_required(VERSION 2.8.3)
project(pddl)


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
  Action.msg
  ActionList.msg
  Problem.msg
  Domain.msg
  LocationBindingList.msg
  LocationBinding.msg
)

add_service_files(
  FILES 
  PlanExecutorSrv.srv 
  PlanGeneratorSrv.srv 
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs gazebo_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)