#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import copy

import rospy
import rospkg
import rosbag
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

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
from sensor_msgs.msg import (
    JointState
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from tf.transformations import *
import baxter_interface
from util.knowledge_base import KnowledgeBase
from util.bayesian_change_point import BayesianChangePoint
from util.general_vis import *
from util.ros_bag import RosBag
from util.file_io import *
from action_primitive_variation.srv import *
from agent.srv import * 
from environment.msg import *


block_bag = RosBag('block')
leftButton_bag = RosBag('leftButton')
rightButton_bag = RosBag('rightButton')
leftGripper_bag = RosBag('leftGripper')
rightGripper_bag = RosBag('rightGripper')
predicates_bag = RosBag('predicate')
jointState_bag = RosBag('jointState')
KB = KnowledgeBase()

shouldRecord = False
vis = True
visNames = []

def handle_APV(req):

    if 'seg' in req.actionName:
        return APVSrvResponse([])
    global shouldRecord

    params = []
    actionToVary = req.actionName
    gripper = req.gripper
    obj = req.obj
    button = req.button

    if gripper is not '': 
        params.append(gripper)
    if obj is not '': 
        params.append(obj)
    if button is not '': 
        params.append(button)

    visName = getVisName(actionToVary + ''.join(params))

    openBags()
    shouldRecord = True
    execute_action(actionToVary, params)
    shouldRecord = False
    changePoints = extract_change_points(gripper, visName, req.clusterThreshold, req.minClusterSize)

    closeBags()

    return APVSrvResponse(changePoints)

def getVisName(nameWithoutIter):
    i = 1
    while(nameWithoutIter + '_' + str(i) in visNames):
        i = i + 1
    return nameWithoutIter + '_' + str(i)

############### START: Call back functions that check to see if ROSbag should be being recorded
def handle_jointStates(data):
    if shouldRecord is not False:
        try:
            jointState_bag.writeToBag('robot/joint_states', data)
        finally:
            pass

def handle_predicates(data):
    if shouldRecord is not False:
        try:
            predicates_bag.writeToBag('predicate_values', data)
        finally:
            pass

def handle_block(data):
    if shouldRecord is not False:
        try:
            block_bag.writeToBag('block_pose', data)
        finally:
            pass

def handle_buttonLeft(data):
    if shouldRecord is not False:
        try:
            leftButton_bag.writeToBag('left_button_pose', data)
        finally:
            pass

def handle_buttonRight(data):
    if shouldRecord is not False:
        try:
            rightButton_bag.writeToBag('right_button_pose', data)
        finally:
            pass

def handle_gripperLeft(data):
    if shouldRecord is not False:
        try:
            leftGripper_bag.writeToBag('left_gripper_pose', data)
            # jointState_bag.writeToBag('left_gripper_pose', data)
        finally:
            pass

def handle_gripperRight(data):
    if shouldRecord is not False:
        try:
            rightGripper_bag.writeToBag('right_gripper_pose', data)
        finally:
            pass
############### END: Call back functions that check to see if ROSbag should be being recorded


def extract_change_points(gripperToConsider, APVtrialName, clustThreshold, minClustSize):
    global visNames
    if gripperToConsider == 'left_gripper':
        bagData = leftGripper_bag.getVisualizableData()
        segs = BayesianChangePoint(np.array(bagData), 'changePointData.csv', clustThreshold, minClustSize)
        cps = segs.getCompressedChangePoints()
        positionInfo = leftGripper_bag.getROSBagDataAtCps(segs.getCompressedChangePoints(), ['left_gripper_pose'], cps)
        if vis == True:
            visData = generateVisData_bagAndCPData('leftGripper', bagData, segs)
            writeBagData(visData, APVtrialName)
            visNames.append(APVtrialName)
        return positionInfo
    else:
        bagData = rightGripper_bag.getVisualizableData()
        segs = BayesianChangePoint(np.array(bagData), 'changePointData.csv', clustThreshold, minClustSize)
        cps = segs.getCompressedChangePoints()
        positionInfo = rightGripper_bag.getROSBagDataAtCps(segs.getCompressedChangePoints(), ['right_gripper_pose'], cps)
        if vis == True:
            visData = generateVisData_bagAndCPData('rightGripper', bagData, segs)
            writeBagData(visData, APVtrialName)
            visNames.append(APVtrialName)
        return positionInfo

def closeBags():
    leftButton_bag.closeBag() 
    rightButton_bag.closeBag() 
    block_bag.closeBag() 
    leftGripper_bag.closeBag() 
    rightGripper_bag.closeBag() 
    jointState_bag.closeBag()

def openBags():
    leftButton_bag.openBag() 
    rightButton_bag.openBag() 
    block_bag.openBag() 
    leftGripper_bag.openBag() 
    rightGripper_bag.openBag() 
    jointState_bag.openBag()

############### END: ROSbag handling


def execute_action(actionName, params):
    b = rospy.ServiceProxy(KB.getService(actionName), KB.getServiceFile(actionName))
    resp = None
    rospy.wait_for_service(KB.getService(actionName), timeout=60)
    try:
        if len(params) == 1:
            resp = b(params[0])
        elif len(params) == 2:
            resp = b(params[0], params[1])   
        elif len(params) == 3:
            resp = b(params[0], params[1], params[2])
        elif len(params) == 4:
            resp = b(params[0], params[1], params[2], params[3])
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)


def main():
    rospy.init_node("APV_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    rospy.Subscriber("left_button_pose", PoseStamped, handle_buttonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, handle_buttonRight)    
    rospy.Subscriber("block_pose", PoseStamped, handle_block)
    rospy.Subscriber("left_gripper_pose", PoseStamped, handle_gripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, handle_gripperRight)
    rospy.Subscriber("robot/joint_states", JointState, handle_jointStates)
    rospy.Subscriber("predicate_values", PredicateList, handle_predicates)

    s = rospy.Service("APV_srv", APVSrv, handle_APV)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
