#!/usr/bin/env python

from __future__ import print_function, division
import roslib
#roslib.load_manifest('my_package')
import sys
import cv2
import math
import time
from cv_bridge import CvBridge, CvBridgeError

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
    GetLinkState,
)
from gazebo_msgs.msg import (
    LinkState,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    PoseArray,
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
    
)
from sensor_msgs.msg import (
    Image,
)
from gazebo_msgs.srv import (
    ApplyJointEffort,
    JointRequest,
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

from environment.srv import *
from environment.msg import *
from util.image_converter import ImageConverter
from util.data_conversion import *

CupPose = None
CoverPose = None
LeftGripperPose = None
RightGripperPose = None
TablePose = None

predicatesPublisher = None 
imageConverter = None 

predicates_list = []


########################################################
def setPoseCup(data):
    global CupPose
    CupPose = data
    updatePredicates("at", "cup", data)

def setPoseCover(data):
    global CoverPose
    CoverPose = data
    updatePredicates("at", "cover", data)

def setPoseGripperLeft(data):
    global LeftGripperPose
    LeftGripperPose = data
    updatePredicates("at", "left_gripper", data)    

def setPoseGripperRight(data):
    global RightGripperPose
    RightGripperPose = data
    updatePredicates("at", "right_gripper", data)    

def setPoseTable(data):
    global TablePose
    TablePose = data
    updatePredicates("at", "table", data)
########################################################


# Jumping off point for updates. "Master" list 
def updatePredicates(oprtr, obj, locInf):
    updateLocationPredicates(oprtr, obj, locInf)
    updateVisionBasedPredicates()
    updatePhysicalStateBasedPredicates()
    predicatesPublisher.publish(predicates_list)

def updateLocationPredicates(oprtr, obj, locInf):
    global predicates_list
    new_predicates = []
    for pred in predicates_list:
        if not((pred.operator == oprtr) and (pred.object == obj)):
            new_predicates.append(pred)
    new_predicates.append(Predicate(operator=oprtr, object=obj, locationInformation=locInf)) 
    predicates_list = new_predicates

def updateVisionBasedPredicates():
    global predicates_list
    new_predicates = []
    for pred in predicates_list:
        if not (pred.operator == "is_visible"):
            new_predicates.append(pred)

    # Need to update the image converter to deal with more objects and to be more sophisticated. 
    # For the image recognition part, every object MUST have a different color to identify it  
    if (imageConverter.getObjectPixelCount('cup') > 0):
        new_predicates.append(Predicate(operator="is_visible", object="cup", locationInformation=None)) 
    if (imageConverter.getObjectPixelCount('cover') > 0):
        new_predicates.append(Predicate(operator="is_visible", object="cover", locationInformation=None)) 
    predicates_list = new_predicates

def updatePhysicalStateBasedPredicates():

    # Physical state choices: touching table, to start 

    global predicates_list
    new_predicates = []
    for pred in predicates_list:
        if ((not (pred.operator == "pressed")) and (not (pred.operator == "obtained"))):
            new_predicates.append(pred)

    if is_touching(LeftGripperPose, TablePose):
        new_predicates.append(Predicate(operator="touching_table", object="left_gripper", locationInformation=None)) 
    if is_touching(RightGripperPose, TablePose):
        new_predicates.append(Predicate(operator="touching_table", object="right_gripper", locationInformation=None)) 
    if is_touching(CupPose, TablePose):
        new_predicates.append(Predicate(operator="touching_table", object="cup", locationInformation=None)) 
    if is_touching(CoverPose, TablePose):
        new_predicates.append(Predicate(operator="touching_table", object="cover", locationInformation=None)) 

    predicates_list = new_predicates

def getPredicates(data):
    return ScenarioDataSrvResponse(pddlStringFormat(predicates_list), 
                                   pddlObjectsStringFormat(predicates_list),
                                   pddlInitStringFormat(predicates_list),
                                   PredicateList(predicates_list))

def main():
    rospy.init_node("scenario_data_node")
    # rospy.wait_for_message("/robot/sim/started", Empty)
   
    global predicatesPublisher 
    global imageConverter 

    predicatesPublisher = rospy.Publisher('predicate_values', PredicateList, queue_size = 10)
    imageConverter = ImageConverter()

    rospy.Subscriber("cup_pose", PoseStamped, setPoseCup)
    rospy.Subscriber("cover_pose", PoseStamped, setPoseCover)
    rospy.Subscriber("left_gripper_pose", PoseStamped, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, setPoseGripperRight)
    rospy.Subscriber("cafe_table_pose", PoseStamped, setPoseTable)

    # rospy.sleep(1)
    s = rospy.Service("scenario_data_srv", ScenarioDataSrv, getPredicates)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

