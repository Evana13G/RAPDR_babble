#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    ApplyJointEffort,
    JointRequest,
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
    Bool,
    Duration,
    Time,
)

def raiseWall():
    rospy.wait_for_service('/gazebo/apply_joint_effort')

    start_time = rospy.Time(0,0)
    duration = rospy.Duration(-1,0)
    
    try:
        wallUp = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        resp_wallUp = wallUp("joint_wall", 10, start_time, duration)
    except rospy.ServiceException, e:
        rospy.logerr("ApplyJointEffort service call failed: {0}".format(e))

def dropWall():
    rospy.wait_for_service('gazebo/clear_joint_forces')

    try:
        wallDown = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        reps_wallDown = wallDown("joint_wall")
    except rospy.ServiceException, e:
        rospy.logerr("JointRequest to clear_joint_forces service call failed: {0}".format(e))