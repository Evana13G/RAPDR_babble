#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg

import tf

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


from agent.srv import *
from environment.srv import ObjectLocationSrv
from util.physical_agent import PhysicalAgent
from util.action_request import ActionRequest
from tf.transformations import *


pa = None
obj_location_srv = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)

################################################################################
#### GENERAL FUNCTIONS #########################################################
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

def rpy_to_quat(roll, pitch, yaw, objPose):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    objPose.pose.orientation.x = quaternion[0]
    objPose.pose.orientation.y = quaternion[1]
    objPose.pose.orientation.z = quaternion[2]
    objPose.pose.orientation.w = quaternion[3]
    return objPose

def orientation_solver(orientationStr, objPose):
    # Obtain main orientation to add offset
    quaternion = (
    objPose.pose.orientation.x,
    objPose.pose.orientation.y,
    objPose.pose.orientation.z,
    objPose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    if (orientationStr == "left_top") or (orientationStr == "right_top"): 
        return objPose
    elif orientationStr == "left_left": 
        roll += -1.5 
        pitch += 1.5 
        return rpy_to_quat(roll, pitch, yaw, objPose)
    elif orientationStr == "left_right":
        pitch += -0.8
        yaw += 1.5
        return rpy_to_quat(roll, pitch, yaw, objPose)
    elif orientationStr == "left_back": 
        pitch += 0.9
        yaw += 0.1
        return rpy_to_quat(roll, pitch, yaw, objPose)
    elif orientationStr == "left_front": 
        pitch += -0.8
        return rpy_to_quat(roll, pitch, yaw, objPose)
    elif orientationStr == "right_left": 
        pitch += 0.8
        yaw += 1.5
        return rpy_to_quat(roll, pitch, yaw, objPose)
    elif orientationStr == "right_right": 
        roll += 1.5 
        pitch += 1.5
        return rpy_to_quat(roll, pitch, yaw, objPose)
    elif orientationStr == "right_back": 
        pitch += 0.8
        return rpy_to_quat(roll, pitch, yaw, objPose)
    elif orientationStr == "right_front": 
        pitch += -0.8
        return rpy_to_quat(roll, pitch, yaw, objPose)
    else:
        return objPose

################################################################################
#### PUSH ######################################################################
def push(req):
    # Pull args
    objPose = getObjectPose(req.objectName)
    start_offset = float(req.startOffset) #FLOAT
    ending_offset = float(req.endOffset) #FLOAT
    rate = req.rate

    # Process args
    obj_y_val = copy.deepcopy(objPose.pose.position.y)  
    startPose = copy.deepcopy(objPose)
    endPose = copy.deepcopy(objPose)
    startPose.pose.position.y = (obj_y_val - start_offset)
    endPose.pose.position.y = (obj_y_val + ending_offset)

    # Put args into hash object
    argNames = ['startPose', 'endPose', 'rate']
    argVals = [startPose, endPose, rate]
    args = arg_list_to_hash(argNames, argVals)

    return PushSrvResponse(pa.push(**args))

################################################################################
#### SHAKE #####################################################################
def shake(req):
    # Pull args
    objPose = getObjectPose(req.objectName)
    limbSide = req.limbSide
    gripperOrientation = req.gripperOrientation
    twist_range = req.twistRange
    rate = req.speed
    num_shakes = req.numShakes

    orientationStr = limbSide + "_" + gripperOrientation
    objPose = orientation_solver(orientationStr, getObjectPose(req.objectName)) # adjust orientation of gripper

    # Put args into hash object
    argNames = ['objPose', 'orientationStr', 'twist_range', 'rate', 'num_shakes']
    argVals = [objPose, orientationStr, twist_range, rate, num_shakes]
    args = arg_list_to_hash(argNames, argVals)

    return ShakeSrvResponse(pa.shake(**args))

################################################################################
#### GRASP #####################################################################
def grasp(req):
    # Pull args
    limbSide = req.limbSide
    gripperOrientation = req.gripperOrientation
    
    orientationStr = limbSide + "_" + gripperOrientation
    objPose = orientation_solver(orientationStr, getObjectPose(req.objectName)) # adjust orientation of gripper

    # Put args into hash object
    argNames = ['objPose', 'orientationStr']
    argVals = [objPose, orientationStr]
    args = arg_list_to_hash(argNames, argVals)
    return GraspSrvResponse(pa.grasp(**args))

################################################################################
#### PRESS #####################################################################
def press(req):
    # Pull args
    objPose = getObjectPose(req.objectName)
    hover_distance = req.hoverDistance
    press_amount = req.pressAmount
    rate = req.rate

    # Process args
    obj_z_val = copy.deepcopy(objPose.pose.position.z)  
    startPose = copy.deepcopy(objPose)
    endPose = copy.deepcopy(objPose)
    startPose.pose.position.z = (obj_z_val + hover_distance)
    endPose.pose.position.z = (obj_z_val + hover_distance - press_amount)

    # Put args into hash object
    argNames = ['startPose', 'endPose', 'rate']
    argVals = [startPose, endPose, rate]
    args = arg_list_to_hash(argNames, argVals)

    return PressSrvResponse(pa.press(**args))

################################################################################
#### DROP ######################################################################
def drop(req):
    # Pull args
    objPose = getObjectPose(req.objectName)
    limbSide = req.limbSide
    gripperOrientation = req.gripperOrientation
    drop_height = req.dropHeight
    
    orientationStr = limbSide + "_" + gripperOrientation
    objPose = orientation_solver(orientationStr, getObjectPose(req.objectName)) # adjust orientation of gripper

    # Put args into hash object
    argNames = ['objPose', 'orientationStr', 'drop_height']
    argVals = [objPose, orientationStr, drop_height]
    args = arg_list_to_hash(argNames, argVals)

    return DropSrvResponse(pa.drop(**args))

################################################################################
def action_executor(req):
    ## To call for any general action to be executed

    actionName = req.actionName
    argNames = req.argNames
    args = req.args # list of strings, should be compatible with action. 
    paramNames = req.paramNames
    paramSettings = req.params # list of floats, should be compatible with action

    assert(len(argNames) == len(args))
    assert(len(paramNames) == len(paramSettings))

    ## Just do push for now 
    a = getCorrectAction(actionName)
    request = ActionRequest(actionName, argNames, args, paramNames, paramSettings)

    a(request)
    return ActionExecutorSrvResponse(1)

################################################################################
## UTIL 
def move_to_start(req):
    return MoveToStartSrvResponse(pa._move_to_start(req.limb))

def open_gripper(req):
    return OpenGripperSrvResponse(pa.gripper_open(req.position))
    
def close_gripper(req):
    return CloseGripperSrvResponse(pa.gripper_close(req.position))

def approach(req):
    return ApproachSrvResponse(pa.approach(req.pose))

def arg_list_to_hash(argNames, argValues):
    args = {}
    for i in range(len(argValues)):
        name = argNames[i]
        val = argValues[i]
        print(name + ': ' + str(val))
        if not(val == 0.0 or val == None or val == 0):
            args[name] = val
    return args

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
    s_9 = rospy.Service("action_executor_srv", ActionExecutorSrv, action_executor)

    rospy.spin()

    return 0 
################################################################################

if __name__ == "__main__":
    main()
