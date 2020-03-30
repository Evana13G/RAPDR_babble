#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from tf.transformations import *

from agent.srv import *
from environment.srv import ObjectLocationSrv
from util.physical_agent import PhysicalAgent

pa = None
obj_location_srv = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)

def getObjectPose(object_name, pose_only=False):
    loc_pStamped = obj_location_srv(object_name)
    if pose_only == True:
        return loc_pStamped.location.pose
    return loc_pStamped.location

def getCorrectAction(action_name):
    action = action_name.split('_')[0]
    actions = {'push' : push,
               'grasp' : grasp, 
               'shake' : shake,
               'press' : press, 
               'drop' : drop}
    return actions[action]
################################################################################

def push(req):
    objPose = getObjectPose(req.objectName)
    start_offset = float(req.startOffset) #FLOAT
    ending_offset = float(req.endOffset) #FLOAT

    obj_y_val = copy.deepcopy(objPose.pose.position.y)  
    startPose = copy.deepcopy(objPose)
    endPose = copy.deepcopy(objPose)
    startPose.pose.position.y = (obj_y_val - start_offset)
    endPose.pose.position.y = (obj_y_val + ending_offset)
    effort = req.rate

    if effort == None or req.rate == 0:
        return PushSrvResponse(pa.push(startPose, endPose, objPose))
    return PushSrvResponse(pa.push(startPose, endPose, objPose, effort))

    # return PushSrvResponse(pa.push(startPose, endPose, objPose))
    # return PushSrvResponse(pa.push_effort(startPose, effort)) 

def grasp(req):
    objPose = getObjectPose(req.objectName)
    return GraspSrvResponse(pa.grasp(objPose))

def shake(req):
    objPose = getObjectPose(req.objectName)
    twist_range = req.twistRange
    speed = req.speed
    return ShakeSrvResponse(pa.shake(objPose, twist_range, speed))

def press(req):
    objPose = getObjectPose(req.objectName)
    hover_distance = req.hoverDistance
    press_amount = req.pressAmount
    return PressSrvResponse(pa.press(objPose, hover_distance, press_amount))

def drop(req):
    objPose = getObjectPose(req.objectName)
    drop_height = req.dropHeight
    return DropSrvResponse(pa.drop(objPose, drop_height))

def action_executor(req):
    ## To call for any genearl action to be executed

    actionName = req.actionName
    argNames = req.argNames
    args = req.args # list of strings, should be compatible with said action. 
    paramNames = req.paramNames
    paramSettings = req.params # list of floats, should be compatible with said action

    assert(len(argNames) == len(args))
    assert(len(paramNames) == len(paramSettings))

    ## Just do push for now 
    a = getCorrectAction(actionName)
    request = ActionRequest(actionName, argNames, args, paramNames, paramSettings)

    a(request)
    return ActionExecutorSrvResponse(1)


class ActionRequest:
    def __init__(self, _actionName, args, argVals, params, paramVals):
        self.actionName = _actionName
        self.initVals(args, argVals, params, paramVals)

    def initVals(self, args, argVals, params, paramVals):
        for i in range(0, len(args)):
            setattr(self, args[i], argVals[i])
        for i in range(0, len(params)):
            setattr(self, params[i], paramVals[i])

    def __str__(self):
        attrs = vars(self)
        s = 'Action Request:\n'
        s = s + '\n'.join("---%s: %s" % item for item in attrs.items())
        return s

################################################################################
## UTIL 
def move_to_start(req):
    return MoveToStartSrvResponse(pa._move_to_start(req.limb))

def set_velocity(req):
    return (pa._set_joint_velocity(req.limb))

def open_gripper(req):
    return OpenGripperSrvResponse(pa.gripper_open(req.position))
    
def close_gripper(req):
    return CloseGripperSrvResponse(pa.gripper_close(req.position))

def approach(req):
    return ApproachSrvResponse(pa.approach(req.pose))
################################################################################

def main():
    rospy.init_node("physical_agent_node")
    rospy.wait_for_service('/object_location_srv')

    global pa
    pa = PhysicalAgent()

    s_1 = rospy.Service("move_to_start_srv", MoveToStartSrv, move_to_start)
    s_2 = rospy.Service("open_gripper_srv", OpenGripperSrv, open_gripper)
    s_2 = rospy.Service("close_gripper_srv", CloseGripperSrv, close_gripper)
    s_3 = rospy.Service("approach_srv", ApproachSrv, approach)

    # Action Primitives
    s_4 = rospy.Service("push_srv", PushSrv, push)
    s_5 = rospy.Service("grasp_srv", GraspSrv, grasp)
    s_6 = rospy.Service("shake_srv", ShakeSrv, shake)
    s_7 = rospy.Service("press_srv", PressSrv, press)
    s_8 = rospy.Service("drop_srv", DropSrv, drop)
    s_9 = rospy.Service("set_joint_velocity_srv", JointVelocitySrv, set_velocity)
    s_9 = rospy.Service("action_executor_srv", ActionExecutorSrv, action_executor)

    rospy.spin()

    return 0 
####################################################################################################

if __name__ == "__main__":
    main()
