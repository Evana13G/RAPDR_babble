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
CoverPose = None
LeftGripperPose = None
RightGripperPose = None
TablePose = None

cover_pressed = False
cup_pressed = False

predicates_list = []

########################################################
def setPoseCup(data):
    global CupPose
    CupPose = data
    updatePredicates("cup", data)

def setPoseCover(data):
    global CoverPose
    CoverPose = data
    updatePredicates("cover", data)

def setPoseGripperLeft(data):
    global LeftGripperPose
    translate(data)
    LeftGripperPose = data
    updatePredicates("left_gripper", data)    

def setPoseGripperRight(data):
    global RightGripperPose
    translate(data)
    RightGripperPose = data
    updatePredicates("right_gripper", data)    

def setPoseTable(data):
    global TablePose
    translate(data, -0.2)
    TablePose = data
    updatePredicates("table", data)

def translate(objPose, z_amt=-1.0):
    objPose.pose.position.z += z_amt

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
    
    # if (imageConverter.getObjectPixelCount('cup') > 0):
    if (imageConverter.is_visible('cup') == True):
        new_predicates.append(Predicate(operator="is_visible", objects=["cup"], locationInformation=None)) 
    # if (imageConverter.getObjectPixelCount('cover') > 0):
    if (imageConverter.is_visible('cover') == True):
        new_predicates.append(Predicate(operator="is_visible", objects=["cover"], locationInformation=None)) 
    predicates_list = new_predicates

def updatePhysicalStateBasedPredicates():
    physical_operators = ['pressed', 'obtained', 'touching'] #grasped?

    # Physical state choices
    global predicates_list
    global cover_pressed

    new_predicates = []
    for pred in predicates_list:
        if pred.operator not in physical_operators:
            new_predicates.append(pred)

    if is_touching(LeftGripperPose, TablePose, 1.0, 0.1):
        new_predicates.append(Predicate(operator="touching", objects=['left_gripper', 'table'], locationInformation=None)) 
    if is_touching(RightGripperPose, TablePose, 1.0, 0.1):
        new_predicates.append(Predicate(operator="touching", objects=['right_gripper', 'table'], locationInformation=None)) 
    if is_touching(CupPose, TablePose, 1.0, 0.1):
        new_predicates.append(Predicate(operator="touching", objects=['cup', 'table'], locationInformation=None)) 
    if is_touching(CoverPose, TablePose, 1.0, 0.1):
        new_predicates.append(Predicate(operator="touching", objects=['cover', 'table'], locationInformation=None)) 
    if is_touching(CoverPose, CupPose, 0.07):
        new_predicates.append(Predicate(operator="touching", objects=['cover', 'cup'], locationInformation=None)) 
    if is_touching(LeftGripperPose, CoverPose, 0.1):
        new_predicates.append(Predicate(operator="touching", objects=['left_gripper', 'cover'], locationInformation=None)) 
    if is_touching(RightGripperPose, CoverPose, 0.1):
        new_predicates.append(Predicate(operator="touching", objects=['right_gripper', 'cover'], locationInformation=None)) 

    if is_pressed(LeftGripperPose, CoverPose, 0.05, [0.01, None]):
        cover_pressed = True
    if is_pressed(RightGripperPose, CoverPose, 0.05, [0.01, None]):
        cover_pressed = True

    if cover_pressed == True:
        new_predicates.append(Predicate(operator="pressed", objects=['cover'], locationInformation=None)) 

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
        'cover': CoverPose,
        'left_gripper': LeftGripperPose,
        'right_gripper': RightGripperPose, 
        'table': TablePose
    }
    return obj_choices.get(obj)

def reset(req):
    global cover_pressed
    cover_pressed = False
    return True

def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_service('is_visible_srv', timeout=60)
   
    rospy.Subscriber("cup_pose", PoseStamped, setPoseCup)
    rospy.Subscriber("cover_pose", PoseStamped, setPoseCover)
    rospy.Subscriber("left_gripper_pose", PoseStamped, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, setPoseGripperRight)
    rospy.Subscriber("cafe_table_pose", PoseStamped, setPoseTable)

    rospy.Service("scenario_data_srv", ScenarioDataSrv, getPredicates)
    rospy.Service("object_location_srv", ObjectLocationSrv, getObjectLocation)
    rospy.Service("reset_env_preds", EmptySrvReq, reset)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

