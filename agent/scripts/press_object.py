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

SRVPROXY_press = rospy.ServiceProxy('press_srv', PressSrv)

def handle_pressObject(req):
    hover_amount = 0.15
    press_amount = 0.01
    SRVPROXY_press(req.objectName, hover_distance, press_amount)
    return PressObjectSrvResponse(1)

def main():
    rospy.init_node("press_object_node")
    s = rospy.Service("press_object_srv", PressObjectSrv, handle_pressObject)
    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())
