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

LeftButtonPose = None
RightButtonPose = None
BlockPose = None
limb = None
button_name = None
poseStampedTo = None

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))
        
def getPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data

def getPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data

def getPoseBlock(data):
    global BlockPose
    BlockPose = data
    
def hoverOverPose(poseStmpd):
	newPose = copy.deepcopy(poseStmpd)
	newPose.pose.position.z += 0.25
	return newPose
	
def grabPose(poseStmpd):
	newPose = copy.deepcopy(poseStmpd)
	newPose.pose.position.z -= 0.02
	return newPose
	

def handle_ObtainObject(req):
    
    global limb
    limb = req.limb
    global object_name
    object_name = req.objectName
    poseTo = None
    
    if object_name == "left_button":
        poseTo = LeftButtonPose
    elif object_name == "right_button":
        poseTo = RightButtonPose
    else:
        poseTo = BlockPose
    
    hover_distance = 0.15

    # Shouldnt have to start at starting pose 
    if limb == 'left_gripper':
		starting_joint_angles_l = {'left_w0': 0.6699952259595108,
								   'left_w1': 1.030009435085784,
                                   'left_w2': -0.4999997247485215,
                                   'left_e0': -1.189968899785275,
                                   'left_e1': 1.9400238130755056,
                                   'left_s0': -0.08000397926829805,
                                   'left_s1': -0.9999781166910306}
    else:
        starting_joint_angles_r = {'right_e0': -0.39888044530362166,
                                   'right_e1': 1.9341522973651006,
                                   'right_s0': 0.936293285623961,
                                   'right_s1': -0.9939970420424453,
                                   'right_w0': 0.27417171168213983,
                                   'right_w1': 0.8298780975195674,
                                   'right_w2': -0.5085333554167599}
    
    currentAction = PhysicalAgent(limb, hover_distance)
    
    if limb == 'left_gripper':
        currentAction.move_to_start(starting_joint_angles_l)
    else:
        currentAction.move_to_start(starting_joint_angles_r)
        
    currentAction.gripper_open()
    currentAction.approach(hoverOverPose(poseTo))
    currentAction.approach(grabPose(poseTo))
    currentAction.gripper_close()
    currentAction.approach(hoverOverPose(poseTo)) 

    if limb == 'left_gripper':
       currentAction.move_to_start(starting_joint_angles_l)
    else:
       currentAction.move_to_start(starting_joint_angles_r)
    
    return ObtainObjectSrvResponse(1)


def main():
    rospy.init_node("obtain_object_node")
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.Subscriber("left_button_pose", PoseStamped, getPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, getPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, getPoseBlock)
    
    s = rospy.Service("obtain_object_srv", ObtainObjectSrv, handle_ObtainObject)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
