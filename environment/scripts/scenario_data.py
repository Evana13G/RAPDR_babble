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

Breakable_Obj_Pose = None

LeftGripperPose = None
RightGripperPose = None
TablePose = None

predicates_list = []

########################################################
def setPoseBreakable_Obj(data):
    global Breakable_Obj_Pose
    Breakable_Obj_Pose = data
    updatePredicates("breakable_obj", data)


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
    
    # if (imageConverter.getObjectPixelCount('breakable_obj') > 0):
    if (imageConverter.is_visible('breakable_obj') == True):
        new_predicates.append(Predicate(operator="is_visible", objects=["breakable_obj"], locationInformation=None)) 


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
    if is_touching(Breakable_Obj_Pose, TablePose):
        new_predicates.append(Predicate(operator="touching", objects=['breakable_obj', 'table'], locationInformation=None)) 

    predicates_list = new_predicates

def getPredicates(data):
    return ScenarioDataSrvResponse(pddlStringFormat(predicates_list), 
                                   pddlObjectsStringFormat(predicates_list),
                                   pddlInitStringFormat(predicates_list),
                                   PredicateList(predicates_list))

def getObjectLocation(data):
    obj = data.obj
    obj_choices = {
        'breakable_obj': Breakable_Obj_Pose,
        'left_gripper': LeftGripperPose,
        'right_gripper': RightGripperPose, 
        'table': TablePose
    }
    return obj_choices.get(obj)

def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_service('is_visible_srv', timeout=60)
   
    rospy.Subscriber("breakable_obj_pose", PoseStamped, setPoseBreakable_Obj)
    rospy.Subscriber("left_gripper_pose", PoseStamped, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, setPoseGripperRight)
    rospy.Subscriber("cafe_table_pose", PoseStamped, setPoseTable)

    rospy.Service("scenario_data_srv", ScenarioDataSrv, getPredicates)
    rospy.Service("object_location_srv", ObjectLocationSrv, getObjectLocation)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

