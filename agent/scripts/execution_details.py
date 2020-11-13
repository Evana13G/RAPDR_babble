#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

import tf

from agent.srv import *
from agent.msg import *
from environment.srv import ObjectLocationSrv
from util.action_request import ActionRequest
from util.data_conversion import arg_list_to_hash
from util.data_conversion import * 

getObjLoc = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)

################################################################################
#### LOCAL INFORMATION #########################################################
offsets = {'cup' : {'left': {'x' : 0, 'y' : 0.13, 'z' : 0},
                    'right': {'x' : 0, 'y' : 0.13, 'z' : 0}, 
                    'top': {'x' : 0, 'y' : 0.13, 'z' : 0}, 
                    'front': {'x' : 0, 'y' : 0.13, 'z' : 0}, 
                    'back': {'x' : 0, 'y' : 0.13, 'z' : 0}}, 
           'cover' :  {'left': {'x' : 0, 'y' : 0.13, 'z' : 0},
                       'right': {'x' : 0, 'y' : 0.13, 'z' : 0}, 
                       'top': {'x' : 0, 'y' : 0.13, 'z' : 0}, 
                       'front': {'x' : 0, 'y' : 0.13, 'z' : 0}, 
                       'back': {'x' : 0, 'y' : 0.13, 'z' : 0}}}
################################################################################
#### ACCESS FUNCTIONS ##########################################################
def getObjectPose(object_name, pose_only=False):
    loc_pStamped = getObjLoc(object_name)
    if pose_only == True:
        return loc_pStamped.location.pose
    return loc_pStamped.location

def setOffset(object_name, start_or_end, val):
    global offsets
    offsets[object_name][start_or_end] = val

def rpy_to_quat(roll, pitch, yaw, objPose):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    objPose.pose.orientation.x = quaternion[0]
    objPose.pose.orientation.y = quaternion[1]
    objPose.pose.orientation.z = quaternion[2]
    objPose.pose.orientation.w = quaternion[3]
    return objPose

################################################################################
#### CALL BACK FUNCTIONS #######################################################

def orientation_solver(req):
    orientationStr = req.gripperName.replace('_gripper', '_') + req.orientation
    objPose = getObjectPose(req.objectName)
    
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


    # Obtain roll, pitch, and yaw for specific orientation 
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

def get_offset(req):
    objectName = req.objectName
    orientation = req.orientation
    vals = offsets[objectName][orientation]
    return HardcodedOffset(vals['x'], vals['y'], vals['z'])

def scenario_settings(req):
    scenario = req.scenario
    if scenario == 'discover_strike':
        coverLoc = getObjLoc('cover').location
        # coverLoc.pose.position.y = coverLoc.pose.position.x + 0.1 # Get this from action movement mag default? 
        # goalLoc = poseStampedToString(coverLoc)                   # ... on second thought, its not necessary. 
        # goal = ['(at cover ' + goalLoc + ')']
        goal = ['(at cover '+ poseStampedToString(coverLoc) + ')']
        orig_scenario = 'default'
        novel_scenario = 'heavy'
        T = 3
        additional_domain_locs = []
    elif scenario == 'discover_pour':
        goal = ['not (touching cup cover)']
        orig_scenario = 'default'
        novel_scenario = 'high_friction'
        T = 3
        additional_domain_locs = []
    else:
        goal = []
        orig_scenario = ''
        novel_scenario = ''
        T = 0
        additional_domain_locs = []
    return GetScenarioSettingsSrvResponse(goal, 
                                          orig_scenario, 
                                          novel_scenario, 
                                          T, 
                                          additional_domain_locs)

################################################################################

def main():
    rospy.init_node("execution_info_node")
    rospy.Service("get_offset", GetHardcodedOffsetSrv, get_offset)
    rospy.Service("calc_gripper_orientation_pose", CalcGripperOrientationPoseSrv, orientation_solver)
    rospy.Service("scenario_settings_srv", GetScenarioSettingsSrv, scenario_settings)
    rospy.spin()

    return 0 
################################################################################

if __name__ == "__main__":
    main()
