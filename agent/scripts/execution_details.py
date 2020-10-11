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

from agent.srv import *
from environment.srv import ObjectLocationSrv
from util.action_request import ActionRequest

obj_location_srv = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)

################################################################################
#### LOCAL INFORMATION #########################################################
offsets = {'cup' : {'start': 0.13,
                    'end' :0.4},
           'cover' : {'start': 0.13,
                    'end' :0.4}}

################################################################################
#### ACCESS FUNCTIONS ##########################################################
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

def getOffset(object_name, start_or_end):
    return offsets[object_name][start_or_end]

def setOffset(object_name, start_or_end, val):
    global offsets
    offsets[object_name][start_or_end] = val

################################################################################
#### PUSH ######################################################################
def push(req):
    objPose = getObjectPose(req.objectName)
    start_offset = getOffset(req.objectName, 'start')
    ending_offset = getOffset(req.objectName, 'end')
    # rate = req.rate
    # # Process args
    # obj_y_val = copy.deepcopy(objPose.pose.position.y)  
    # startPose = copy.deepcopy(objPose)
    # endPose = copy.deepcopy(objPose)
    # startPose.pose.position.y = (obj_y_val - start_offset)
    # endPose.pose.position.y = (obj_y_val + ending_offset)

    # # Put args into hash object
    # argNames = ['startPose', 'endPose', 'rate']
    # argVals = [startPose, endPose, rate]
    # args = arg_list_to_hash(argNames, argVals)

    return 1

################################################################################
#### SHAKE #####################################################################
def shake(req):
    # # Pull args
    # objPose = getObjectPose(req.objectName)
    # twist_range = req.twistRange
    # rate = req.speed

    # # Put args into hash object
    # argNames = ['objPose', 'twist_range', 'rate']
    # argVals = [objPose, twist_range, rate]
    # args = arg_list_to_hash(argNames, argVals)

    return 1

################################################################################
#### GRASP #####################################################################
def grasp(req):
    # # Pull args
    # objPose = getObjectPose(req.objectName)
    return 1

################################################################################
#### PRESS #####################################################################
def press(req):
    # # Pull args
    # objPose = getObjectPose(req.objectName)
    # hover_distance = req.hoverDistance
    # press_amount = req.pressAmount
    # rate = req.rate

    # # Process args
    # obj_z_val = copy.deepcopy(objPose.pose.position.z)  
    # startPose = copy.deepcopy(objPose)
    # endPose = copy.deepcopy(objPose)
    # startPose.pose.position.z = (obj_z_val + hover_distance)
    # endPose.pose.position.z = (obj_z_val + hover_distance - press_amount)

    # # Put args into hash object
    # argNames = ['startPose', 'endPose', 'rate']
    # argVals = [startPose, endPose, rate]
    # args = arg_list_to_hash(argNames, argVals)
    
    return 1

################################################################################
#### DROP ######################################################################
def drop(req):
    # # Pull args
    # objPose = getObjectPose(req.objectName)
    # drop_height = req.dropHeight


    # # Process args
    # obj_z_val = copy.deepcopy(objPose.pose.position.z)  
    # dropPose = copy.deepcopy(objPose)
    # dropPose.pose.position.z = (obj_z_val + drop_height)

    # # Put args into hash object
    # argNames = ['objPose', 'dropPose']
    # argVals = [objPose, dropPose]
    # args = arg_list_to_hash(argNames, argVals)

    return 1

################################################################################
def execution_info(req):
    ## To call for any general action to be executed

    actionName = req.actionName
    argNames = req.argNames

    a = getCorrectAction(actionName)
    request = ActionRequest(actionName, argNames)

    a(request)

    # return ActionExecutorSrvResponse(1)
    return 1

def arg_list_to_hash(argNames, argValues):
    args = {}
    for i in range(len(argValues)):
        name = argNames[i]
        val = argValues[i]
        if not(val == 0.0 or val == None or val == 0):
            args[name] = val
    return args

################################################################################

def main():
    rospy.init_node("execution_info_node")
    # rospy.wait_for_service('/object_location_srv')

    s = rospy.Service("get_execution_info", GetExecutionInfoSrv, execution_info)


    rospy.spin()

    return 0 
################################################################################

if __name__ == "__main__":
    main()
