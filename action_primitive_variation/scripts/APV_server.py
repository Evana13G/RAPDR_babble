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
from util.file_io import *
from util.general_vis import *
from action_primitive_variation.srv import *
from pddl.srv import *
from agent.srv import * 
from environment.msg import *
from environment.srv import *

objLocProxy = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)
actionExecutorProxy = rospy.ServiceProxy('action_executor_srv', ActionExecutorSrv)
visSrvProxy = rospy.ServiceProxy('record_limb_data_srv', RecordLimbDataSrv) 
addBreakptSrvProxy = rospy.ServiceProxy('add_action_breakpt_srv', AddActionBreakptSrv) 
actionInfoProxy = rospy.ServiceProxy('get_KB_action_info_srv', GetKBActionInfoSrv)
envResetProxy = rospy.ServiceProxy('load_environment', HandleEnvironmentSrv)

def getObjectPose(object_name, pose_only=False):
    loc_pStamped = obj_location_srv(object_name)
    if pose_only == True:
        return loc_pStamped.location.pose
    return loc_pStamped.location

def handle_APV(req):
    # if '_' in req.actionName:
    #     change the action name to have just the original action

    actionToVary = req.actionName
    args = req.args
    paramToVary = req.param
    T = req.T 

    if paramToVary == None or paramToVary == '':
        return APVSrvResponse([])

    # This is where you should pull the param names and args from KB
    actionInfo = actionInfoProxy(actionToVary).actionInfo
    argNames = actionInfo.executableArgNames
    paramNames = actionInfo.paramNames
    paramDefaults = list(actionInfo.paramDefaults)
    paramMins = list(actionInfo.paramMins)
    paramMaxs = list(actionInfo.paramMaxs)
    
    # preconditions = actionInfo.preconditions
    # effects = actionInfo.effects
    
    assert(len(argNames) == len(args))
    assert(len(paramNames) == len(paramDefaults) == len(paramMins) == len(paramMaxs))

    i_paramToVary = paramNames.index(paramToVary)
    paramMin = paramMins[i_paramToVary]
    paramMax = paramMaxs[i_paramToVary]
    I = (paramMax - paramMin)/T

    ## Process parameter values 
    paramVals = []
    for i in range(0, T-1):
        addition =  i * I
        paramVals.append(paramMin + addition)
    paramVals.append(paramMax)

    for paramAssignment in paramVals:
        paramSettings = copy.deepcopy(paramDefaults)
        paramSettings[i_paramToVary] = paramAssignment
        print('Action: ' + str(actionToVary) + ', Param: ' + str(paramToVary) + ', ' + str(paramAssignment))
        actionExecutorProxy(actionToVary, argNames, args, paramNames, paramSettings)
        envResetProxy('restart', 'heavy')
    return APVSrvResponse([])


def main():
    rospy.init_node("APV_node")
    rospy.wait_for_service('/action_executor_srv')
    s = rospy.Service("APV_srv", APVSrv, handle_APV)
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())

    # # visSrvProxy('start', '')
    # for paramAssignment in paramVals:
    #     # addBreakptSrvProxy() 
    #     # envResetProxy('restart', 'default') # action, environment setting
    #     paramSettings = copy.deepcopy(paramDefaults)
    #     paramSettings[i_paramToVary] = paramAssignment
    #     print('Action: ' + str(actionToVary) + ', Param: ' + str(paramToVary) + ', ' + str(paramAssignment))
    #     actionExecutorProxy(actionToVary, argNames, args, paramNames, paramSettings)
        
    #     envResetProxy('restart', 'default')
    #     # addBreakptSrvProxy() 
    # # visSrvProxy('end', 'endejeje')
    # return APVSrvResponse([])
