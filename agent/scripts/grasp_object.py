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

from tf.transformations import *

from agent.srv import *

SRVPROXY_grasp = rospy.ServiceProxy('grasp_srv', GraspSrv)

def handle_graspObject(req):
    SRVPROXY_grasp(req.objectName)
    return GraspObjectSrvResponse(1)

def main():
    rospy.init_node("grasp_object_node")
    s = rospy.Service("grasp_object_srv", GraspObjectSrv, handle_graspObject)
    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())
