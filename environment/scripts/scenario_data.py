#!/usr/bin/env python

import rospy


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

import baxter_interface

from environment.srv import *
from environment.msg import *
from util.image_converter import ImageConverter
from util.data_conversion import *

predicatesPublisher = rospy.Publisher('predicate_values', PredicateList, queue_size = 10)
imageConverter = ImageConverter()
# isVisible = rospy.ServiceProxy('is_visible_srv', IsVisibleSrv)

CupPose = None
MarblePose = None
LeftGripperPose = None
RightGripperPose = None
TablePose = None

predicates_list = []

########################################################
def setPoseCup(data):
    global CupPose
    CupPose = data
    updatePredicates("plastic_cup", data)

def setPoseCover(data):
    global MarblePose
    MarblePose = data
    updatePredicates("marble_1_5cm", data)

def setPoseGripperLeft(data):
    global LeftGripperPose
    LeftGripperPose = data
    updatePredicates("left_gripper", data)    

def setPoseGripperRight(data):
    global RightGripperPose
    RightGripperPose = data
    updatePredicates("right_gripper", data)    

def setPoseTable(data):
    global TablePose
    TablePose = data
    updatePredicates("table", data)
########################################################

# Jumping off point for updates. "Master" list 
def updatePredicates(obj, locInf):
    updateLocationPredicates("at", obj, locInf)
    updateVisionBasedPredicates()
    updatePhysicalStateBasedPredicates()
    predicatesPublisher.publish(predicates_list)

def updateLocationPredicates(oprtr, obj, locInf):
    global predicates_list
    new_predicates = []
    for pred in predicates_list:
        if not((pred.operator == oprtr) and (obj in pred.objects)): ## This needs to change
            new_predicates.append(pred)
    new_predicates.append(Predicate(operator=oprtr, objects=[obj], locationInformation=locInf)) 
    predicates_list = new_predicates

def updateVisionBasedPredicates():
    global predicates_list
    new_predicates = []
    for pred in predicates_list:
        if not (pred.operator == "is_visible"):
            new_predicates.append(pred)

    # Need to update the image converter to deal with more objects and to be more sophisticated. 
    # For the image recognition part, every object MUST have a different color to identify it  
<<<<<<< HEAD
    if (imageConverter.getObjectPixelCount('plastic_cup') > 0):
        new_predicates.append(Predicate(operator="is_visible", objects=["cup"], locationInformation=None)) 
    if (imageConverter.getObjectPixelCount('marble_1_5cm') > 0):
=======
    
    # if (imageConverter.getObjectPixelCount('cup') > 0):
    if (imageConverter.is_visible('cup') == True):
        new_predicates.append(Predicate(operator="is_visible", objects=["cup"], locationInformation=None)) 
    # if (imageConverter.getObjectPixelCount('cover') > 0):
    if (imageConverter.is_visible('cover') == True):
>>>>>>> 76f96f72725437586d99ae5a1f8560a3d162eb84
        new_predicates.append(Predicate(operator="is_visible", objects=["cover"], locationInformation=None)) 
    predicates_list = new_predicates

def updatePhysicalStateBasedPredicates():
    physical_operators = ['pressed', 'obtained', 'touching'] #grasped?

    # Physical state choices
    global predicates_list
    new_predicates = []
    for pred in predicates_list:
        if pred.operator not in physical_operators:
            new_predicates.append(pred)

    if is_touching(LeftGripperPose, TablePose):
        new_predicates.append(Predicate(operator="touching", objects=['left_gripper', 'table'], locationInformation=None)) 
    if is_touching(RightGripperPose, TablePose):
        new_predicates.append(Predicate(operator="touching", objects=['right_gripper', 'table'], locationInformation=None)) 
    if is_touching(CupPose, TablePose):
        new_predicates.append(Predicate(operator="touching", objects=['cup', 'table'], locationInformation=None)) 
    if is_touching(MarblePose, TablePose):
        new_predicates.append(Predicate(operator="touching", objects=['cover', 'table'], locationInformation=None)) 
    if is_touching(MarblePose, CupPose, 0.235):
        new_predicates.append(Predicate(operator="touching", objects=['cover', 'cup'], locationInformation=None)) 
    if is_touching(LeftGripperPose, MarblePose, 1.1):
        new_predicates.append(Predicate(operator="touching", objects=['left_gripper', 'cover'], locationInformation=None)) 
    if is_touching(RightGripperPose, MarblePose, 1.1):
        new_predicates.append(Predicate(operator="touching", objects=['right_gripper', 'cover'], locationInformation=None)) 

    predicates_list = new_predicates

def getPredicates(data):
    return ScenarioDataSrvResponse(pddlStringFormat(predicates_list), 
                                   pddlObjectsStringFormat(predicates_list),
                                   pddlInitStringFormat(predicates_list),
                                   PredicateList(predicates_list))

def getObjectLocation(data):
    obj = data.obj
    obj_choices = {
        'cup': CupPose,
        'marble_1_5cm': MarblePose,
        'left_gripper': LeftGripperPose,
        'right_gripper': RightGripperPose, 
        'table': TablePose
    }
    return obj_choices.get(obj)

def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_service('is_visible_srv', timeout=60)
   
    rospy.Subscriber("cup_pose", PoseStamped, setPoseCup)
    rospy.Subscriber("marble_1_5cm", PoseStamped, setPoseCover)
    rospy.Subscriber("left_gripper_pose", PoseStamped, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, setPoseGripperRight)
    rospy.Subscriber("cafe_table_pose", PoseStamped, setPoseTable)

    rospy.Service("scenario_data_srv", ScenarioDataSrv, getPredicates)
    rospy.Service("object_location_srv", ObjectLocationSrv, getObjectLocation)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

