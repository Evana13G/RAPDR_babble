#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np
import os 

import rospy
import rospkg
import rosbag

from std_msgs.msg import (
    Header,
    Empty,
)

from action_primitive_variation.srv import *
from agent.srv import * 
from pddl.msg import *
from pddl.srv import *

actionExecutorProxy = rospy.ServiceProxy('action_executor_srv', ActionExecutorSrv)
actionInfoProxy = rospy.ServiceProxy('get_KB_action_info_srv', GetKBActionInfoSrv)

def handle_plan(req):
    try:
        for action in req.actions.actions:
            actionName = action.actionName
            args = action.argVals  

            actionInfo = actionInfoProxy(actionName).actionInfo

            argNames = actionInfo.executableArgNames
            paramNames = actionInfo.paramNames
            paramDefaults = actionInfo.paramDefaults
            assert(len(argNames) == len(args))

            actionExecutorProxy(actionName, argNames, args, paramNames, paramDefaults)
        
        return PlanExecutorSrvResponse(1)        
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return PlanExecutorSrvResponse(0)

def main():
    rospy.init_node("plan_executor_node")

    s = rospy.Service("plan_executor_srv", PlanExecutorSrv, handle_plan)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
