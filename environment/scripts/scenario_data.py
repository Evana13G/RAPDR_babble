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

from std_msgs.msg import Bool

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
BurnerPose = None
RightButtonPose = None
LeftButtonPose = None
# Breakable_Obj_Pose = None

cover_pressed = False
cup_pressed = False
require_burner_on = False
left_button_pressed = False
right_button_pressed = False


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

def setPoseBurner(data):
    global BurnerPose
    # translate(data, -0.2)
    BurnerPose = data
    updatePredicates("burner1", data)

def setPoseLeftButton(data):
    global LeftButtonPose
    LeftButtonPose = data
    updatePredicates("left_button", data)

def setPoseRightButton(data):
    global RightButtonPose
    RightButtonPose = data
    updatePredicates("right_button", data)

# def setPoseBreakable_Obj(data):
#     global Breakable_Obj_Pose
#     Breakable_Obj_Pose = data
#     updatePredicates("breakable_obj", data)

def set_require_burner_on(data):
    global require_burner_on
    require_burner_on = data

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
    if (imageConverter.is_visible('cup') == True):
        new_predicates.append(Predicate(operator="is_visible", objects=["cup"], locationInformation=None)) 
    if (imageConverter.is_visible('cover') == True):
        new_predicates.append(Predicate(operator="is_visible", objects=["cover"], locationInformation=None)) 
    
    predicates_list = new_predicates

def updatePhysicalStateBasedPredicates():
    physical_operators = ['pressed', 'obtained', 'touching', 'on', 'on_burner', 'covered', 'cooking', 'powered_on'] #grasped?

    # Physical state choices
    global predicates_list
    global left_button_pressed
    global right_button_pressed

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
    if is_touching(CoverPose, CupPose, 0.1):
        new_predicates.append(Predicate(operator="touching", objects=['cover', 'cup'], locationInformation=None)) 
    if is_touching(LeftGripperPose, CoverPose, 0.1):
        new_predicates.append(Predicate(operator="touching", objects=['left_gripper', 'cover'], locationInformation=None)) 
    if is_touching(RightGripperPose, CoverPose, 0.1):
        new_predicates.append(Predicate(operator="touching", objects=['right_gripper', 'cover'], locationInformation=None)) 

    if is_pressed(LeftGripperPose, LeftButtonPose, 0.08, [0.07, None]):
        left_button_pressed = True
    if is_pressed(RightGripperPose, LeftButtonPose, 0.08, [0.07, None]):
        left_button_pressed = True
    if is_pressed(LeftGripperPose, RightButtonPose, 0.08, [0.07, None]):
        right_button_pressed = True
    if is_pressed(RightGripperPose, RightButtonPose, 0.08, [0.07, None]):
        right_button_pressed = True


    item_on_burner = []
    covered_item = []
    burner_on = False

    if is_touching(CoverPose, BurnerPose, 0.1, 0.06):
        new_predicates.append(Predicate(operator="touching", objects=['cover', 'burner1'], locationInformation=None)) 
        new_predicates.append(Predicate(operator="on_burner", objects=['cover', 'burner1'], locationInformation=None)) 
        item_on_burner.append('cover')

    if is_touching(CupPose, BurnerPose, 0.1, 0.06):
        new_predicates.append(Predicate(operator="touching", objects=['cup', 'burner1'], locationInformation=None)) 
        new_predicates.append(Predicate(operator="on_burner", objects=['cup', 'burner1'], locationInformation=None)) 
        item_on_burner.append('cup')

    if is_pressed(CupPose, CoverPose, 0.1, [0.07, None]):
        new_predicates.append(Predicate(operator="covered", objects=['cover'], locationInformation=None)) 
        covered_item.append('cover')

    if is_pressed(CoverPose, CupPose, 0.1, [0.07, None]):
        new_predicates.append(Predicate(operator="covered", objects=['cup'], locationInformation=None)) 
        covered_item.append('cup')

    if right_button_pressed == True:
        new_predicates.append(Predicate(operator="pressed", objects=['left_button'], locationInformation=None)) 
        new_predicates.append(Predicate(operator="powered_on", objects=['burner1'], locationInformation=None)) 
        burner_on = True

    if right_button_pressed == True:
        new_predicates.append(Predicate(operator="pressed", objects=['right_button'], locationInformation=None)) 

    if item_on_burner is not []:
        for item in item_on_burner:
            if (item in covered_item) == True:
                if require_burner_on == False:
                    new_predicates.append(Predicate(operator="cooking", objects=[item], locationInformation=None)) 
                elif burner_on == True:
                    new_predicates.append(Predicate(operator="cooking", objects=[item], locationInformation=None)) 

    predicates_list = new_predicates


def getPredicates(data):
    return ScenarioDataSrvResponse(pddlStringFormat(predicates_list), 
                                   pddlObjectsStringFormat(predicates_list),
                                   pddlInitStringFormat(predicates_list),
                                   PredicateList(predicates_list))

def getObjectLocation(data):
    obj = data.obj
    obj_choices = {
        # 'breakable_obj': Breakable_Obj_Pose,
        'cup': CupPose,
        'cover': CoverPose,
        'left_gripper': LeftGripperPose,
        'right_gripper': RightGripperPose, 
        'table': TablePose,
        'burner1': BurnerPose,
        'left_button': LeftButtonPose,
        'right_button': RightButtonPose
    }
    return obj_choices.get(obj)

def reset(req):
    global left_button_pressed
    global right_button_pressed
    left_button_pressed = False
    right_button_pressed = False
    return True

def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_service('is_visible_srv', timeout=60)
   
    # rospy.Subscriber("breakable_obj_pose", PoseStamped, setPoseBreakable_Obj)
    rospy.Subscriber("left_gripper_pose", PoseStamped, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, setPoseGripperRight)
    rospy.Subscriber("cafe_table_pose", PoseStamped, setPoseTable)
    rospy.Subscriber("burner1_pose", PoseStamped, setPoseBurner)
    rospy.Subscriber("cup_pose", PoseStamped, setPoseCup)
    rospy.Subscriber("cover_pose", PoseStamped, setPoseCover)
    rospy.Subscriber("left_button_pose", PoseStamped, setPoseLeftButton)
    rospy.Subscriber("right_button_pose", PoseStamped, setPoseRightButton)
    rospy.Subscriber("require_burner_on", Bool, set_require_burner_on)

    rospy.Service("scenario_data_srv", ScenarioDataSrv, getPredicates)
    rospy.Service("object_location_srv", ObjectLocationSrv, getObjectLocation)
    rospy.Service("reset_env_preds", EmptySrvReq, reset)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

