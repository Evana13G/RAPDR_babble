cmake_minimum_required(VERSION 2.8.3)
project(agent)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

#add_message_files(
#  FILES
#)

add_service_files(
FILES 
ApproachSrv.srv
MoveToStartSrv.srv
GraspSrv.srv
CloseGripperSrv.srv 
OpenGripperSrv.srv 
BrainSrv.srv
PushSrv.srv 
PushObjectSrv.srv 
ShakeSrv.srv
PressSrv.srv
DropSrv.srv
JointVelocitySrv.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

