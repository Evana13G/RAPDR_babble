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

add_message_files(
FILES
HardcodedOffset.msg
SuccessAction.msg
)

add_service_files(
FILES 
MoveToStartSrv.srv
JointVelocitySrv.srv
ActionExecutorSrv.srv
CalcGripperOrientationPoseSrv.srv
GetHardcodedOffsetSrv.srv
EmptyTestSrv.srv
PddlExecutorSrv.srv
RawActionExecutorSrv.srv
ParamActionExecutorSrv.srv
IsVisibleSrv.srv
BrainSrv.srv
GetScenarioSettingsSrv.srv
GetMoveMagUnitSrv.srv
RunExperimentSrv.srv
GetScenarioGoalSrv.srv
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
