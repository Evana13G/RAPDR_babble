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

SRVPROXY_push = rospy.ServiceProxy('push_srv', PushSrv)

def handle_pushObject(req):
    start_offset = -0.1
    end_offset = 0.15
    SRVPROXY_push(req.objectName, start_offset, end_offset)

    return PushObjectSrvResponse(1)


def main():
    rospy.init_node("push_object_node")

    s = rospy.Service("push_object_srv", PushObjectSrv, handle_pushObject)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
