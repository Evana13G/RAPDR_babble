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

PlasticCupPose = None
MarbleRPose = None
MarbleBPose = None
LeftGripperPose = None
RightGripperPose = None
TablePose = None

predicates_list = []

########################################################
def setPosePlasticCup(data):
    global PlasticCupPose
    PlasticCupPose = data
    updatePredicates("plastic_cup", data)

def setPoseMarbleB(data):
    global MarbleBPose
    MarbleBPose = data
    updatePredicates("marbleB", data)

def setPoseMarbleR(data):
    global MarbleRPose
    MarbleRPose = data
    updatePredicates("marbleR", data)

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
    if (imageConverter.getObjectPixelCount('plastic_cup') > 0):
        new_predicates.append(Predicate(operator="is_visible", objects=["plastic_cup"], locationInformation=None)) 
    if (imageConverter.getObjectPixelCount('marbleB') > 0):
        new_predicates.append(Predicate(operator="is_visible", objects=["marbleB"], locationInformation=None))
    if (imageConverter.getObjectPixelCount('marbleR') > 0):
        new_predicates.append(Predicate(operator="is_visible", objects=["marbleR"], locationInformation=None)) 
    if (imageConverter.is_visible('plastic_cup') == True):
        new_predicates.append(Predicate(operator="is_visible", objects=["plastic_cup"], locationInformation=None)) 
    if (imageConverter.is_visible('cover') == True):
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

    # if is_touching(LeftGripperPose, TablePose):
    #     new_predicates.append(Predicate(operator="touching", objects=['left_gripper', 'table'], locationInformation=None)) 
    # if is_touching(RightGripperPose, TablePose):
    #     new_predicates.append(Predicate(operator="touching", objects=['right_gripper', 'table'], locationInformation=None)) 
    # if is_touching(CupPose, TablePose):
    #     new_predicates.append(Predicate(operator="touching", objects=['cup', 'table'], locationInformation=None)) 
    # if is_touching(CoverPose, TablePose):
    #     new_predicates.append(Predicate(operator="touching", objects=['cover', 'table'], locationInformation=None)) 
    # if is_touching(CoverPose, CupPose):
    #     new_predicates.append(Predicate(operator="touching", objects=['cover', 'cup'], locationInformation=None)) 
    # if is_touching(LeftGripperPose, CoverPose, 1.1):
    #     new_predicates.append(Predicate(operator="touching", objects=['left_gripper', 'cover'], locationInformation=None)) 
    # if is_touching(RightGripperPose, MarbleBPose, 1.1):
    #     new_predicates.append(Predicate(operator="touching", objects=['right_gripper', 'cover'], locationInformation=None)) 
    # if is_touching(MarbleRPose, TablePose):
    #     new_predicates.append(Predicate(operator="touching", objects=['marbleR', 'table'], locationInformation=None)) 
    # if is_touching(MarbleRPose, PlasticCupPose, 0.235):
    #     new_predicates.append(Predicate(operator="touching", objects=['marbleR', 'plastic_cup'], locationInformation=None)) 
    # if is_touching(LeftGripperPose, MarbleRPose, 1.1):
    #     new_predicates.append(Predicate(operator="touching", objects=['left_gripper', 'marbleR'], locationInformation=None)) 
    # if is_touching(RightGripperPose, MarbleRPose, 1.1):
    #     new_predicates.append(Predicate(operator="touching", objects=['right_gripper', 'marbleR'], locationInformation=None))        

    predicates_list = new_predicates

def getPredicates(data):
    return ScenarioDataSrvResponse(pddlStringFormat(predicates_list), 
                                   pddlObjectsStringFormat(predicates_list),
                                   pddlInitStringFormat(predicates_list),
                                   PredicateList(predicates_list))

def getObjectLocation(data):
    obj = data.obj
    obj_choices = {
        'plastic_cup': PlasticCupPose,
        'marbleB': MarbleBPose,
        'marbleR': MarbleRPose,
        'left_gripper': LeftGripperPose,
        'right_gripper': RightGripperPose, 
        'table': TablePose
    }
    return obj_choices.get(obj)

def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_service('is_visible_srv', timeout=60)
   
    rospy.Subscriber("plastic_cup_pose", PoseStamped, setPosePlasticCup)
    rospy.Subscriber("marbleB", PoseStamped, setPoseMarbleB)
    rospy.Subscriber("marbleR", PoseStamped, setPoseMarbleR)
    rospy.Subscriber("left_gripper_pose", PoseStamped, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, setPoseGripperRight)
    rospy.Subscriber("cafe_table_pose", PoseStamped, setPoseTable)

    rospy.Service("scenario_data_srv", ScenarioDataSrv, getPredicates)
    rospy.Service("object_location_srv", ObjectLocationSrv, getObjectLocation)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

