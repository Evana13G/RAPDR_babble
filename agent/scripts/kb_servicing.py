#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg

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
from util.knowledge_base import KnowledgeBase

KB = KnowledgeBase()
    
def get_loc(req):
    return GetActionLocSrvResponse(KB.getAction(req.actionName).getLocs())
	
# def add_action(req):
#     return AddActionSrvResponse(1)

def main():
    rospy.init_node("knowledge_base_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
    s1 = rospy.Service("get_action_loc_srv", GetActionLocSrv, get_loc)
    # s2 = rospy.Service("add_action_srv", AddActionSrv, add_action)

    rospy.spin()    
    return 0

if __name__ == '__main__':
    sys.exit(main())
