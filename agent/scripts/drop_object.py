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


SRVPROXY_drop = rospy.ServiceProxy('drop_srv', DropSrv)

def handle_dropObject(req):

    obj = req.objectName
    
    # PARAM TO BE VARIED
    drop_height = 0.15

    SRVPROXY_drop(obj, drop_height)

    return DropObjectSrvResponse(1)

def main():
    rospy.init_node("drop_object_node")
    s = rospy.Service("drop_object_srv", DropObjectSrv, handle_dropObject)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
