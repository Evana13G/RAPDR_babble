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
from util.physical_agent import PhysicalAgent

SRVPROXY_shake = rospy.ServiceProxy('shake_srv', ShakeSrv)

def handle_shakeObject(req):

    twist_range = 1
    speed = 1

    SRVPROXY_shake(req.objectName, twist_range, speed)

    return ShakeObjectSrvResponse(1)

def main():
    rospy.init_node("shake_object_node")

    s = rospy.Service("shake_object_srv", ShakeObjectSrv, handle_shakeObject)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
