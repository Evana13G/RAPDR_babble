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
from geometry_msgs.msg import (
    PoseStamped,
    # Pose,
    # Point,
    # Quaternion,
)
from tf.transformations import *

from util.knowledge_base import KnowledgeBase
from util.data_conversion import is_touching
from action_primitive_variation.srv import *
from agent.srv import * 
from util.physical_agent import PhysicalAgent
from pddl.srv import *

actionToVary = None 
gripper = None
obj = None
button = None

# AP_names = ['press_button', 'obtain_object']
# AP_services = ['press_button_srv', 'obtain_object_srv']
# AP_srvs = [PressButtonSrv, ObtainObjectSrv]

KB = KnowledgeBase()

lPA = None
rPA = None 

def execute_action(req):

    pose_initial = req.initPosition
    pose_final = req.endPosition
    if 'left' in req.limb:
        pa = lPA
    else:
        pa = rPA

    pa.approach(pose_initial)

    if not is_touching(pose_initial, pose_final): # Maybe check distance between poses to check if they're close enough
        print("checked to see if the start and end pos were equal")
        pa.approach(pose_final)

    try:
        return PartialPlanExecutorSrvResponse(1)
    except rospy.ServiceException, e:
        return PartialPlanExecutorSrvResponse(0)
        print("Service call failed: %s"%e)


def main():

    rospy.init_node("partial_plan_executor_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
    
    global lPA
    global rPA
    
    lPA = PhysicalAgent(limb='left_gripper', hover_distance=0.0) # 
    rPA = PhysicalAgent(limb='right_gripper', hover_distance=0.0) # 

    s = rospy.Service("partial_plan_executor_srv", PartialPlanExecutorSrv, execute_action) #rewrite
    



    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
