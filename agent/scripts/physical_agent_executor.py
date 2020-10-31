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


from agent.srv import *
from pddl.srv import *
from pddl.msg import *
from environment.srv import ObjectLocationSrv
from util.physical_agent import PhysicalAgent
from util.action_request import ActionRequest
from util.data_conversion import arg_list_to_hash

pa = None
obj_location_srv = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)
actionInfoProxy = rospy.ServiceProxy('get_KB_action_info_srv', GetKBActionInfoSrv)

def getOffset(object, orientation):
    rospy.wait_for_service('get_offset')
    try:
        query = rospy.ServiceProxy('get_offset', GetHardcodedOffsetSrv)
        response = query(object, orientation)
        return response.hardcodings
    except rospy.ServiceException as e:
        print("Service call to get_offset failed: %s"%e)

def getObjectPose(object_name, pose_only=False):
    loc_pStamped = obj_location_srv(object_name)
    if pose_only == True:
        return loc_pStamped.location.pose
    return loc_pStamped.location

def getCorrectAction(action_name):
    action = action_name.split('_')[0]
    actions = {'push' : push,
               # 'grasp' : grasp, 
               'shake' : shake,
               'press' : press}
               # 'drop' : drop}
    return actions[action]

################################################################################
################################################################################

#### PUSH ######################################################################


def push(req):
    print(req)
    gripper = req.gripper
    objPose = getObjectPose(req.objectName)
    rate = req.rate
    
    
    ending_offset = req.movementMagnitude
    orientation = req.orientation
    start_offset = getOffset(req.objectName, orientation).y

    # Process args
    startPose = copy.deepcopy(objPose)
    endPose = copy.deepcopy(objPose)

    if orientation == 'left':
        obj_y_val = copy.deepcopy(objPose.pose.position.y) 
        startPose.pose.position.y = (obj_y_val - start_offset)
        endPose.pose.position.y = (obj_y_val + ending_offset)
    elif orientation == 'right':
        obj_y_val = copy.deepcopy(objPose.pose.position.y)  
        startPose.pose.position.y = (obj_y_val + start_offset)
        endPose.pose.position.y = (obj_y_val - ending_offset)


    # Put args into hash object
    argNames = ['gripper', 'startPose', 'endPose', 'rate']
    argVals = [gripper, startPose, endPose, rate]
    args = arg_list_to_hash(argNames, argVals)

    return pa.push(**args)

#### SHAKE #####################################################################
def shake(req):
    gripper = req.gripper
    objPose = getObjectPose(req.objectName)
    rate = req.rate
    twist_range = req.movementMagnitude
    orientation = req.orientation

    argNames = ['gripper', 'objPose', 'twist_range', 'rate']
    argVals = [gripper, objPose, twist_range, rate]
    args = arg_list_to_hash(argNames, argVals)

    return pa.shake(**args)

#### PRESS #####################################################################
def press(req):
    gripper = req.gripper
    objPose = getObjectPose(req.objectName)
    rate = req.rate
    press_amount = req.movementMagnitude
    orientation = req.orientation

    hover_distance = 0.1
    
    # Process args
    obj_z_val = copy.deepcopy(objPose.pose.position.z)  
    startPose = copy.deepcopy(objPose)
    endPose = copy.deepcopy(objPose)
    startPose.pose.position.z = (obj_z_val + hover_distance)
    endPose.pose.position.z = (obj_z_val + hover_distance - press_amount)

    # Put args into hash object
    argNames = ['gripper', 'startPose', 'endPose', 'rate']
    argVals = [gripper, startPose, endPose, rate]
    args = arg_list_to_hash(argNames, argVals)

    return pa.press(**args)


#### DROP ######################################################################
# def drop(req):
#     # Pull args
#     gripper = req.gripper
#     objPose = getObjectPose(req.objectName)
#     drop_height = req.dropHeight
#     # Process args
#     obj_z_val = copy.deepcopy(objPose.pose.position.z)  
#     dropPose = copy.deepcopy(objPose)
#     dropPose.pose.position.z = (obj_z_val + drop_height)
#     # Put args into hash object
#     argNames = ['objPose', 'dropPose']
#     argVals = [objPose, dropPose]
#     args = arg_list_to_hash(argNames, argVals)
#     return ActionExecutorSrvResponse(pa.drop(**args))

# #### GRASP #####################################################################
# def grasp(req):
#     gripper = req.gripper
#     objPose = getObjectPose(req.objectName)
#     return ActionExecutorSrvResponse(pa.grasp(objPose))

################################################################################
################################################################################

## To call for any general action to be executed
# Performs checks and send to the appropriate srv
def action_executor(req):
    assert(len(req.argNames) == len(req.args))
    assert(len(req.paramNames) == len(req.params))

    a = getCorrectAction(req.actionName)
    zipped_request = ActionRequest(req.actionName, 
                                   req.argNames, 
                                   req.args, 
                                   req.paramNames, 
                                   req.params)
    a(zipped_request)

def raw_action_executor(req):
    actionName = req.actionName
    args = req.argVals
    params = req.params 

    actionInfo = actionInfoProxy(actionName).actionInfo
    # sets params
    argNames = actionInfo.executableArgNames
    paramNames = actionInfo.paramNames

    assert(len(argNames) == len(args))
    assert(len(paramNames) == len(params))

    action_executor(Action(actionName, argNames, paramNames, args, params))
    return RawActionExecutorSrvResponse(1)

def param_action_executor(req):
    actionName = req.actionName
    argValues = req.argVals
    paramNamesToSet = req.paramNames 
    paramValsToSet = req.paramVals 

    actionInfo = actionInfoProxy(actionName).actionInfo
    argNames = actionInfo.executableArgNames
    paramNames = actionInfo.paramNames
    paramVals = actionInfo.paramDefaults

    assert(len(argNames) == len(argValues))
    assert(len(paramNamesToSet) == len(paramValsToSet))

    for i in range(len(paramValsToSet)):
        pNameToSet = paramNamesToSet[i]
        pValToSet = paramValsToSet[i]
        i_pToSet = paramNames.index(pNameToSet)
        paramVals[i_pToSet] = pValToSet

    action_executor(Action(actionName, argNames, paramNames, argValues, paramVals))
    return ParamActionExecutorSrvResponse(1)

# This just takes in one action, pulls param values, and sends to the 
# Appropriate srv, which takes care of the hardcodings call. 
def pddl_action_executor(req):
    actionName = req.actionName
    args = req.argVals

    actionInfo = actionInfoProxy(actionName).actionInfo
    argNames = actionInfo.executableArgNames
    paramNames = actionInfo.paramNames
    paramDefaults = actionInfo.paramDefaults

    assert(len(argNames) == len(args))
    action_executor(Action(actionName, argNames, paramNames, args, paramDefaults))
    return PddlExecutorSrvResponse(1)

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

################################################################################

def main():
    rospy.init_node("physical_agent_node")
    rospy.wait_for_service('/object_location_srv')

    global pa
    pa = PhysicalAgent()

    rospy.Service("move_to_start_srv", MoveToStartSrv, move_to_start)
    rospy.Service("pddl_action_executor_srv", PddlExecutorSrv, pddl_action_executor)
    rospy.Service("raw_action_executor_srv", RawActionExecutorSrv, raw_action_executor)
    rospy.Service("param_action_executor_srv", ParamActionExecutorSrv, param_action_executor)

    rospy.spin()

    return 0 
################################################################################

if __name__ == "__main__":
    main()
