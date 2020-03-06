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
from util.physical_agent import PhysicalAgent

pa = None

CupPose = None
CoverPose = None
LeftGripper = None

def setPoseCup(data):
    global CupPose
    CupPose = data
    
def setPoseCover(data):
    global CoverPose
    CoverPose = data

def setLeftGripperPose(data):
    global LeftGripper
    LeftGripper = data

def getObjectPose(obj, poseStamped=True):
    if poseStamped == True:
        if obj == 'cup': 
            poseTo = CupPose
        elif obj == 'cover':
            poseTo = CoverPose
        else:
            poseTo = LeftGripper
    else:
        if obj == 'cup': 
            poseTo = CupPose.pose
        elif obj == 'cover':
            poseTo = CoverPose.pose
        else:
            poseTo = LeftGripper.pose
    return poseTo

################################################################################
def move_to_start(req):
    return MoveToStartSrvResponse(pa._move_to_start(req.limb))

def set_velocity(req):
    print("Success")
    return (pa._set_joint_velocity(req.limb))

def open_gripper(req):
    return OpenGripperSrvResponse(pa.gripper_open(req.position))
    
def close_gripper(req):
    return CloseGripperSrvResponse(pa.gripper_close(req.position))

def approach(req):
    return ApproachSrvResponse(pa.approach(req.pose))

################################################################################

def push(req):
    objPose = getObjectPose(req.objectName)
    start_offset = req.startOffset #FLOAT
    ending_offset = req.endOffset #FLOAT
    obj_y_val = copy.deepcopy(objPose.pose.position.y)
    startPose = copy.deepcopy(objPose)
    endPose = copy.deepcopy(objPose)

    startPose.pose.position.y = (obj_y_val - start_offset)
    endPose.pose.position.y = (obj_y_val + ending_offset)

    return PushSrvResponse(pa.push(startPose, endPose, objPose))


def grasp(req):
    objPose = getObjectPose(req.objectName)
    return GraspSrvResponse(pa.grasp(objPose))

def shake(req):
    objPose = getObjectPose(req.objectName)
    twist_range = req.twistRange
    speed = req.speed
    return ShakeSrvResponse(pa.shake(objPose, twist_range, speed))

def press(req):
    objPose = getObjectPose(req.objectName)
    hover_distance = req.hoverDistance
    press_amount = req.pressAmount
    return PressSrvResponse(pa.press(objPose, hover_distance, press_amount))

def drop(req):
    objPose = getObjectPose(req.objectName)
    drop_height = req.dropHeight
    return DropSrvResponse(pa.drop(objPose, drop_height))


def main():
    rospy.init_node("physical_agent_node")

    global pa
    pa = PhysicalAgent()

    rospy.Subscriber("cover_pose", PoseStamped, setPoseCover)
    rospy.Subscriber("cup_pose", PoseStamped, setPoseCup)
    rospy.Subscriber("left_gripper_pose", PoseStamped, setLeftGripperPose)

    s_1 = rospy.Service("move_to_start_srv", MoveToStartSrv, move_to_start)
    s_2 = rospy.Service("open_gripper_srv", OpenGripperSrv, open_gripper)
    s_2 = rospy.Service("close_gripper_srv", CloseGripperSrv, close_gripper)
    s_3 = rospy.Service("approach_srv", ApproachSrv, approach)

    # Action Primitives
    s_4 = rospy.Service("push_srv", PushSrv, push)
    s_5 = rospy.Service("grasp_srv", GraspSrv, grasp)
    s_6 = rospy.Service("shake_srv", ShakeSrv, shake)
    s_7 = rospy.Service("press_srv", PressSrv, press)
    s_8 = rospy.Service("drop_srv", DropSrv, drop)
    s_9 = rospy.Service("set_joint_velocity_srv", JointVelocitySrv, set_velocity)

    # s_9 = rospy.Service("push_object_srv")

    rospy.spin()

    return 0 
####################################################################################################

if __name__ == "__main__":
    main()
