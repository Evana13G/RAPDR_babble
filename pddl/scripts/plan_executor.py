#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np
import os 
import copy

import rospy
import rospkg
import rosbag

from std_msgs.msg import (
    Header,
    Empty,
)

from tf.transformations import *

from util.knowledge_base import KnowledgeBase
from action_primitive_variation.srv import *
from agent.srv import * 
from pddl.msg import *
from pddl.srv import *

actionToVary = None 
gripper = None
obj = None
button = None


AP_names = ['press_button', 'obtain_object']
AP_services = ['press_button_srv', 'obtain_object_srv']
AP_srvs = [PressButtonSrv, ObtainObjectSrv]

KB = KnowledgeBase()

def handle_plan(req):
    try:
        for action in req.actions.actions:
            execute_action(action.name, action.params, action.motionPoints)
        return PlanExecutorSrvResponse(1)        
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return PlanExecutorSrvResponse(0)

############### START: ROSbag handling


def execute_action(actionName, params, endEffectorList):
    if 'seg' in actionName:   
        b = rospy.ServiceProxy('partial_plan_executor_srv', PartialPlanExecutorSrv)
        rospy.wait_for_service('partial_plan_executor_srv', timeout=60)
        #endEffectors = actionName.split('.')
        resp = None
        try: # You do need params 
            resp = b(params[0], endEffectorList[0], endEffectorList[1]) 
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
    else:
        b = rospy.ServiceProxy(KB.getService(actionName), KB.getServiceFile(actionName))
        rospy.wait_for_service(KB.getService(actionName), timeout=60)
        resp = None
        
        try:
            resp = b(params[0], params[2])   
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)


def main():
    rospy.init_node("plan_executor_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    # rospy.Subscriber("left_button_pose", PoseStamped, handle_buttonLeft)

    s = rospy.Service("plan_executor_srv", PlanExecutorSrv, handle_plan)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
