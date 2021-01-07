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
  ActionInfo.msg
  ActionExecutionInfo.msg
  ActionExecutionInfoList.msg
  Problem.msg
  Domain.msg
  LocationBindingList.msg
  LocationBinding.msg
  ExecutableAction.msg
  ActionPDDLBinding.msg
  PlanExecutionOutcome.msg
  DiscreteParamVals.msg
)

add_service_files(
  FILES 
  PlanExecutorSrv.srv 
  PlanGeneratorSrv.srv 
  GetKBActionLocsSrv.srv
  GetKBDomain.srv
  GetKBPddlLocsSrv.srv
  GetKBActionInfoSrv.srv
  GetActionPDDLBindingSrv.srv
  AddActionToKBSrv.srv
  CheckEffectsSrv.srv
  NovelEffectsSrv.srv
  GetParamOptionsSrv.srv
  RemoveActionFromKBSrv.srv
  ResetKBSrv.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs gazebo_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)