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

from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)

from tf.transformations import *

import baxter_interface
from util.knowledge_base import KnowledgeBase
from util.file_io import *
from util.general_vis import *
from action_primitive_variation.srv import *
from agent.srv import * 
from environment.msg import *
from environment.srv import ObjectLocationSrv

KB = KnowledgeBase()
objLocProxy = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)
actionExecutorProxy = rospy.ServiceProxy('action_executor_srv', ActionExecutorSrv)

# pushProxy = rospy.ServiceProxy('push_srv', PushSrv)
# graspProxy = rospy.ServiceProxy('grasp_srv', PushSrv)
# shakeProxy = rospy.ServiceProxy('shake_srv', PushSrv)
# pressProxy = rospy.ServiceProxy('press_srv', PushSrv)
# dropProxy = rospy.ServiceProxy('drop_srv', PushSrv)


position = []
orientation = []
linear = []
angular = []
force = []
torque = []

l_gripper_l_finger_joint_VELOCITY = []
l_gripper_r_finger_joint_VELOCITY = []
left_e0_VELOCITY = []
left_e1_VELOCITY = []
left_s0_VELOCITY = []
left_s1_VELOCITY = []
left_w0_VELOCITY = []
left_w1_VELOCITY = []
left_w2_VELOCITY = []

l_gripper_l_finger_joint_EFFORT = []
l_gripper_r_finger_joint_EFFORT = []
left_e0_EFFORT = []
left_e1_EFFORT = []
left_s0_EFFORT = []
left_s1_EFFORT = []
left_w0_EFFORT = []
left_w1_EFFORT = []
left_w2_EFFORT = []

# startingTimesteps = []

def getObjectPose(object_name, pose_only=False):
    loc_pStamped = obj_location_srv(object_name)
    if pose_only == True:
        return loc_pStamped.location.pose
    return loc_pStamped.location

def handle_APV(req):
    # if '_' in req.actionName:
    #     change the action name to have just the original action

    # This is where you should pull the param names and args from KB
    # actionToVary = req.actionName
    # argNames = KB.getAction(actionToVary).getArgNames(actionToVary)
    # args = req.args
    # paramNames = [req.param]
    # paramVals = [0]
    # T = req.T 
    # paramMin = KB.getAction(actionToVary).getParamMin(paramToVary)
    # paramMax = KB.getAction(actionToVary).getParamMax(paramToVary)
    # I = (paramMax - paramMin)/T

    actionToVary = req.actionName
    
    argNames = ['gripper', 'objectName', 'startOffset', 'endOffset']
    args = req.args

    paramNames = [req.param]
    
    T = req.T 
    paramMin = 0.0
    paramMax = 500.0
    I = (paramMax - paramMin)/T

    paramVals = []
    for i in range(0, T-1):
        addition =  i * I
        paramVals.append(paramMin + addition)
    paramVals.append(paramMax)

    for paramAssignment in paramVals:
        actionExecutorProxy(actionToVary, argNames, args, paramNames, [paramAssignment])
                # string actionName
                # string[] argNames
                # string[] args
                # string[] paramNames
                # float64[] params
    print("APV Complete")

    visData()
    return APVSrvResponse([])

def print_data():
    print("Position: " + str(position[0]))
    print("Orientation: " + str(orientation[0]))
    print("Linear: " + str(linear[0]))
    print("Angular: " + str(angular[0]))
    print("Force: " + str(force[0]))
    print("Torque: " + str(torque[0]))

def visData():
    # generateEndptImage(position, 'position')
    # generateEndptImage(orientation, 'orientation')
    # generateEndptImage(linear, 'linear')
    # generateEndptImage(angular, 'angular')
    # generateEndptImage(force, 'force')
    # generateEndptImage(torque, 'torque')
    visData = [l_gripper_l_finger_joint_VELOCITY, l_gripper_r_finger_joint_VELOCITY, left_e0_VELOCITY, left_e1_VELOCITY, left_s0_VELOCITY, left_s1_VELOCITY, left_w0_VELOCITY, left_w1_VELOCITY, left_w2_VELOCITY]
    visLabels = ['l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    visTitle = 'VELOCITIES'
    generateJointImage(visData, visLabels, visTitle)

    visData = [l_gripper_l_finger_joint_EFFORT, l_gripper_r_finger_joint_EFFORT, left_e0_EFFORT, left_e1_EFFORT, left_s0_EFFORT, left_s1_EFFORT, left_w0_EFFORT, left_w1_EFFORT, left_w2_EFFORT]
    visTitle = 'EFFORTS'
    generateJointImage(visData, visLabels, visTitle)

def recordData(msg):
    global position
    global orientation
    global linear
    global angular
    global force
    global torque

    position.append(msg.pose.position)
    orientation.append(msg.pose.orientation)
    linear.append(msg.twist.linear)
    angular.append(msg.twist.angular)
    force.append(msg.wrench.force)
    torque.append(msg.wrench.torque)

def recordJSData(msg):
    global l_gripper_l_finger_joint_VELOCITY
    global l_gripper_r_finger_joint_VELOCITY
    global left_e0_VELOCITY
    global left_e1_VELOCITY
    global left_s0_VELOCITY
    global left_s1_VELOCITY
    global left_w0_VELOCITY
    global left_w1_VELOCITY
    global left_w2_VELOCITY

    global l_gripper_l_finger_joint_EFFORT
    global l_gripper_r_finger_joint_EFFORT
    global left_e0_EFFORT
    global left_e1_EFFORT
    global left_s0_EFFORT
    global left_s1_EFFORT
    global left_w0_EFFORT
    global left_w1_EFFORT
    global left_w2_EFFORT

    l_gripper_l_finger_joint_VELOCITY.append(msg.velocity[1])
    l_gripper_r_finger_joint_VELOCITY.append(msg.velocity[2])
    left_e0_VELOCITY.append(msg.velocity[3])
    left_e1_VELOCITY.append(msg.velocity[4])
    left_s0_VELOCITY.append(msg.velocity[5])
    left_s1_VELOCITY.append(msg.velocity[6])
    left_w0_VELOCITY.append(msg.velocity[7])
    left_w1_VELOCITY.append(msg.velocity[8])
    left_w2_VELOCITY.append(msg.velocity[9])

    l_gripper_l_finger_joint_EFFORT.append(msg.effort[1])
    l_gripper_r_finger_joint_EFFORT.append(msg.effort[2])
    left_e0_EFFORT.append(msg.effort[3])
    left_e1_EFFORT.append(msg.effort[4])
    left_s0_EFFORT.append(msg.effort[5])
    left_s1_EFFORT.append(msg.effort[6])
    left_w0_EFFORT.append(msg.effort[7])
    left_w1_EFFORT.append(msg.effort[8])
    left_w2_EFFORT.append(msg.effort[9])
    
# name: [head_pan, l_gripper_l_finger_joint, l_gripper_r_finger_joint, left_e0, left_e1, left_s0,
#   left_s1, left_w0, left_w1, left_w2, r_gripper_l_finger_joint, r_gripper_r_finger_joint,
#   right_e0, right_e1, right_s0, right_s1, right_w0, right_w1, right_w2]
# position: [-0.00023099540588589207, -1.903265869904104e-10, -8.856709743036387e-06, -1.189354599009981, 1.9390110969187369, -0.07832617126942854, -0.9980960350480785, 0.6713189034896159, 1.0298541057770096, -0.4912958852473448, -2.616245090759819e-07, -0.02083300024170109, -0.3972688084751841, 1.9281250052225722, 0.9312897053704257, -0.9855728914442894, 0.27272192866291967, 0.8264580962372019, -0.5063068312893062]
# velocity: [-4.975790835185836e-08, 4.882642922386573e-08, 1.6425445220141443e-06, 9.406894506457359e-08, -2.643932357611246e-07, -1.1223301788411846e-07, 3.7882927501185685e-08, -2.7873097975806554e-07, 5.632818225923825e-07, 7.189502491699974e-07, -0.00025936768129211407, 2.2666591930239824e-06, 1.93544229988058e-06, -1.8880351835825666e-08, -3.943475773774642e-07, -1.8965163663281878e-07, 2.2026330941221007e-05, 1.2368330097190778e-05, -1.2393053596697799e-05]
# effort: [0.0, 2.368648170365627e-07, 0.008858334025296141, -0.12454053711419277, -0.1612792239114036, 2.2553092726695922e-06, -0.1596223360422755, 0.00796568910889306, -0.005018994534395915, -3.1116328003122362e-06, 0.0, 0.0, -0.051736134610180784, -0.10612135054177685, 4.4309333979697385e-06, -0.2375401326926152, -0.009218293307640124, 0.020535497472664588, 0.000838518725974069]


def main():
    rospy.init_node("APV_node")
    rospy.wait_for_service('/action_executor_srv')

    s = rospy.Service("APV_srv", APVSrv, handle_APV)
    
    recordDataSubscriber = rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, recordData)
    recordJSSubscriber = rospy.Subscriber('/robot/joint_states', JointState, recordJSData)
    

    rospy.spin()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
