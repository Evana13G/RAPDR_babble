#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import copy

import rospy
import rospkg
import rosbag
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

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
from sensor_msgs.msg import (
    JointState
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)

from tf.transformations import *

import baxter_interface
from util.knowledge_base import KnowledgeBase
from util.file_io import *
from util.general_vis import *
from action_primitive_variation.srv import *
from agent.srv import * 
from environment.msg import *
from environment.srv import ObjectLocationSrv

KB = KnowledgeBase()
objLocProxy = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)
actionExecutorProxy = rospy.ServiceProxy('action_executor_srv', ActionExecutorSrv)

def getObjectPose(object_name, pose_only=False):
    loc_pStamped = obj_location_srv(object_name)
    if pose_only == True:
        return loc_pStamped.location.pose
    return loc_pStamped.location

def handle_APV(req):
    # if '_' in req.actionName:
    #     change the action name to have just the original action

    # This is where you should pull the param names and args from KB
    # actionToVary = req.actionName
    # argNames = KB.getAction(actionToVary).getArgNames(actionToVary)
    # args = req.args
    # paramNames = [req.param]
    # paramVals = [0]
    # T = req.T 
    # paramMin = KB.getAction(actionToVary).getParamMin(paramToVary)
    # paramMax = KB.getAction(actionToVary).getParamMax(paramToVary)
    # I = (paramMax - paramMin)/T

    actionToVary = req.actionName
    
    argNames = ['gripper', 'objectName', 'startOffset', 'endOffset']
    args = req.args

    paramNames = [req.param]
    
    T = req.T 
    paramMin = 0.0
    paramMax = 500.0
    I = (paramMax - paramMin)/T

    paramVals = []
    for i in range(0, T-1):
        addition =  i * I
        paramVals.append(paramMin + addition)
    paramVals.append(paramMax)

    indices = []

    for paramAssignment in paramVals:
        actionExecutorProxy(actionToVary, argNames, args, paramNames, [paramAssignment])
        indices.append(len(l_gripper_l_finger_joint_VELOCITY))

    print("APV Complete")

    visData(indices)

    return APVSrvResponse([])

def main():
    rospy.init_node("APV_node")
    rospy.wait_for_service('/action_executor_srv')

    s = rospy.Service("APV_srv", APVSrv, handle_APV)

    rospy.spin()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
