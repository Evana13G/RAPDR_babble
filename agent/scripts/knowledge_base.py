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
from util.knowledge_base.knowledge_base import KnowledgeBase


def handle_KB_req(req):
    print(KB.getDomainData())
    return 1

def get_actions(req): 
    actions = KB.getActions()
    for action in actions:
        print(action)
    return 1 

################################################################################

def main():
    rospy.init_node("knowledge_base_node")

    global KB
    KB = KnowledgeBase()

    s1 = rospy.Service("get_KB_data", GetKBDataSrv, handle_KB_req)
    s2 = rospy.Service("get_KB_actions", GetKBDataSrv, get_actions)

    rospy.spin()

    return 0 
################################################################################

if __name__ == "__main__":
    main()
