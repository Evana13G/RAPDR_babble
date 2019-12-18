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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from tf.transformations import *

import baxter_interface

from agent.srv import *
from util.physical_agent import PhysicalAgent

pa = None

def move_to_start(req):
    return MoveToStartSrvResponse(pa._move_to_start(req.limb))

def toggle_gripper_state(req):
    if req.desiredState == 'open':
        return ToggleGripperStateSrvResponse(pa._gripper_open(req.gripperName))
    else:
        return ToggleGripperStateSrvResponse(pa._gripper_close(req.gripperName))

def approach(req):
    return ApproachSrvResponse(pa._approach(req.gripperName, req.pose))


def main():
    rospy.init_node("physical_agent_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
    global pa
    pa = PhysicalAgent()
    s_1 = rospy.Service("move_to_start_srv", MoveToStartSrv, move_to_start)
    s_2 = rospy.Service("toggle_gripper_state_srv", ToggleGripperStateSrv, toggle_gripper_state)
    s_3 = rospy.Service("approach_srv", ApproachSrv, approach)

    rospy.spin()

    return 0 
####################################################################################################

if __name__ == "__main__":
    main()
