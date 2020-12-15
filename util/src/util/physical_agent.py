#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg

import time

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    ApplyJointEffort,
    ApplyBodyWrench,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Wrench,
    Vector3,
)
from std_msgs.msg import (
    Header,
    Empty,
    Time, 
    Duration,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from tf.transformations import *

import baxter_interface

##################################################################

class PhysicalAgent(object):
    def __init__(self, hover_distance = 0.1, verbose=False):
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._left_limb = baxter_interface.Limb('left')
        self._right_limb = baxter_interface.Limb('right')
        self._left_gripper = baxter_interface.Gripper('left')
        self._right_gripper = baxter_interface.Gripper('right')
        
        ns_left = "ExternalTools/left/PositionKinematicsNode/IKService"
        ns_right = "ExternalTools/right/PositionKinematicsNode/IKService"

        self._iksvc_left = rospy.ServiceProxy(ns_left, SolvePositionIK)
        self._iksvc_right = rospy.ServiceProxy(ns_right, SolvePositionIK)

        self._joint_effort_svc = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        self._body_wrench_svc = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        rospy.wait_for_service(ns_left, 5.0)
        rospy.wait_for_service(ns_right, 5.0)

        if self._verbose:
            print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        if self._verbose:
            print("Enabling robot... ")
        self._rs.enable()

        # self.orientationSolver = rospy.ServiceProxy("calc_gripper_orientation_pose", CalcGripperOrientationPoseSrv)

        self._pos_offsets_dict = dict({'right_front0': [0, 0.01, 0.075],
                                        'right_front1': [0.015, 0.005, -0.08],
                                        'right_left0': [0, 0.04, 0],
                                        'right_left1': [0, -0.05, 0],
                                        'right_back0': [0.02, 0.005, 0],
                                        'right_back1': [-0.02, 0, 0],
                                        'right_right0': [-0.03, -0.01, -0.015],
                                        'right_right1': [-0.038, 0.05, 0],
                                        'right_top0': [0, 0, 0.1],
                                        'right_top1': [0, 0, -0.09],
                                        'left_front0': [0, -0.01, 0.075],
                                        'left_front1': [0.015, -0.01, -0.08],
                                        'left_left0': [-0.055, 0.1, -0.01],
                                        'left_left1': [0, -0.15, 0],
                                        'left_back0': [0.005, 0.005, 0],
                                        'left_back1': [0, 0, 0],
                                        'left_right0': [-0.1, -0.02, -0.015],
                                        'left_right1': [0.01, 0.115],
                                        'left_top0': [0, 0, 0],
                                        'left_top1': [0, 0, 0]})

####################################################################################################
############## Higher Level Action Primitives 

    # def push(self, startPose, endPose, rate=100):
    #     self._gripper_close("left")
    #     self._hover_approach("left", startPose)
    #     self._approach("left", startPose)
    #     self._approach("left", endPose, rate=rate)
    #     self._retract("left")
    #     return 1

    def push(self, gripper, startPose, endPose, rate=10):
        gripper_name = gripper.replace('_gripper', '')
        limb = self.translateLimb(gripper_name)
        self._gripper_close(gripper_name)
        self._hover_approach(gripper_name, startPose)
        self._approach(gripper_name, startPose)

        ########### Add code here 
        joint_angles_start = self.ik_request(gripper_name, startPose)
        joint_angles_end = self.ik_request(gripper_name, endPose)

        joint_movement_amounts = {}
        # T = 1.0/float(rate)
        if gripper_name == 'left':
            joints = ['left_w0','left_w1','left_w2','left_e0','left_e1','left_s0','left_s1']
        else:
            joints = ['right_w0','right_w1','right_w2','right_e0','right_e1','right_s0','right_s1']

        for joint in joints: 
            diff = joint_angles_end[joint] - joint_angles_start[joint]
            joint_movement_amounts[joint] = diff*float(rate)

        _rate = 1000.0
        missed_cmds = 20.0
        control_rate = rospy.Rate(_rate)
        limb.set_command_timeout((1.0 / _rate) * missed_cmds)

        i = 0
        while i<500:
            limb.set_joint_velocities(joint_movement_amounts) 
            control_rate.sleep()
            i = i + 1
        
        rospy.sleep(1)
        self._retract(gripper_name)
        # return 1

    def grasp(self, gripper, objPose, orientation='left'):
        # Note: For left gripper the initial object dimensions was 0.4 by 0.4 
        # The right gripper does not open as far as the left gripper, so 0.25 by 0.25 is used 

        gripper_name = gripper.replace('_gripper', '')
        print(gripper_name)
        self._gripper_open(gripper_name)

        # Approach two different positions for smooth grasp action
        orientationStr = gripper_name + '_' + orientation

        appr = copy.deepcopy(objPose)

        for i in range(0,1):
            pos = self._pos_offsets_dict[orientationStr + str(i)]
            appr.pose.position.x += pos[0]
            appr.pose.position.y += pos[1]
            appr.pose.position.z += pos[2]
            self._approach(gripper_name, appr)

        self._gripper_close(gripper_name)

    def shake(self, gripper, objPose, orientation='left', twist_range=1, rate=10.0):
        # For now, assume left gripper is moving (change to an argument)
        # Number of times to shake can be adjusted by the for loop 

        # twist_range tells you how much to move in each direction (delta)
        # speed has to do with the duration of the sleep between the two positions (frequency)

        time_steps = 20
        rate = 1.0/rate

        gripper_name = gripper.replace('_gripper', '')
        limb = self.translateLimb(gripper_name)

        limb_joints = limb.joint_names()
        joint_name= limb_joints[6] # gripper twist, left_w2

        # GRIP OBJECT 
        # self._gripper_open(gripper_name)
        # self._hover_approach(gripper_name, objPose)
        # self._approach(gripper_name, objPose)
        # self._gripper_close(gripper_name)
        # self._hover_approach(gripper_name, objPose)

        self.grasp(gripper_name, objPose, orientation)
        self._hover_approach(gripper_name, objPose) # increases z position to lift object up

        begin_position = limb.joint_angle(joint_name) # pose robot will move to at the end

        # Gripper moves in between two positions
        for i in range(time_steps):
            current_position = limb.joint_angle(joint_name)
            joint_command = {joint_name: current_position + twist_range}
            limb.set_joint_positions(joint_command)
            time.sleep(rate)
            current_position = limb.joint_angle(joint_name)
            joint_command = {joint_name: current_position - twist_range}
            limb.set_joint_positions(joint_command)
            time.sleep(rate)
            
        joint_command = {joint_name: begin_position}
        limb.set_joint_positions(joint_command)
        self._approach(gripper_name, objPose)
        self._gripper_open(gripper_name)
        self._hover_approach(gripper_name, objPose)
        # return 1

    def press(self, gripper, startPose, endPose, rate=100): 
        gripper_name = gripper.replace('_gripper', '')
        self._gripper_close(gripper_name)
        self._approach(gripper_name, startPose)
        self._approach(gripper_name, endPose, rate=rate)
        self._retract(gripper_name)
        # return 1

    def drop(self, gripper, objPose, dropPose):
        gripper_name = gripper.replace('_gripper', '')
        self._gripper_open(gripper_name)
        self._hover_approach(gripper_name, objPose)
        self._approach(gripper_name, objPose)
        self._gripper_close(gripper_name)
        self._approach(gripper_name, dropPose)
        self._gripper_open(gripper_name)
        # return 1

###################################################################################################
############## Lower Level Action Primitives 

    def _gripper_open(self, gripperName):
        try:
            (self.translateGripper(gripperName)).open()
            rospy.sleep(1.0)
            return 1
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 0

    def _gripper_close(self, gripperName):
        try:
            (self.translateGripper(gripperName)).close()
            rospy.sleep(1.0)
            return 1
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 0

    ####### ADDED by Amel 
    def _set_joint_velocity(self, limb='both'):
        velocities_l = {'left_w0' :  4.067311133343405e-050,
                        'left_w1' : 1.6401705346185865e-080,
                        'left_w2' : 5.840094167063631e-060}
        velocities_r = {'right_w0' :  4.067311133343405e-050,
                        'right_w1' : 1.6401705346185865e-080,
                        'right_w2' : 5.840094167063631e-060}

        try:
            if limb == 'left_gripper':
                _limb = self.translateLimb(limb)
                _limb.set_joint_velocities(velocities_l)
            elif limb == 'right_gripper':
                _limb = self.translateLimb(limb)
                _limb.set_joint_velocities(velocities_r)
            else:
                _limb = self.translateLimb('left_gripper')
                _limb.set_joint_velocities(velocities_l)
                _limb = self.translateLimb('right_gripper')
                _limb.set_joint_velocities(velocities_r)

            rospy.sleep(2.0)

            return 1
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 0

    def _set_joint_efforts(self):

        # print(self._left_limb.endpoint_effort())
        # efforts = []
        # start_time = rospy.Time(0.0)
        # start_time = rospy.Time.now()
        # duration = rospy.Duration(5000.0)
        # torques = {'head_pan' : 0.0, 
        #            'l_gripper_l_finger_joint' : -0.035200525640670714,
        #            'l_gripper_r_finger_joint' : -0.25935460745992633, 
        #            'left_e0' : 47.77774881151675, 
        #            'left_e1' : 23.51433066190589, 
        #            'left_s0' : 11.794216672654034, 
        #            'left_s1' : 23.60884891094095, 
        #            'left_w0' : -10.76398527560869, 
        #            'left_w1' : 15.0, 
        #            'left_w2' : -0.8971144799225961, 
        #            'r_gripper_l_finger_joint' : 0.0, 
        #            'r_gripper_r_finger_joint' : 0.0, 
        #            'right_e0' : -0.05173647335432463, 
        #            'right_e1' : -0.10614423477850465, 
        #            'right_s0' : 0.00023955770158679002, 
        #            'right_s1' : -0.23776845263334678, 
        #            'right_w0' : -0.009212185201725731, 
        #            'right_w1' : 0.020528554873848748, 
        #            'right_w2' : 0.0008220324551100333}
        # l_torques = {'left_w0' : -10.76398527560869, 
        #              'left_w1' : 15.0, 
        #              'left_w2' : -0.8971144799225961}

        # self._left_limb.set_joint_torques(l_torques)


        self._left_limb._cartesian_effort= {
            'force': Point(0.0,0.0,0.0),
            'torque': Point(10.0, 0.0, 0.0),
        }

        rospy.sleep(2.0)

    ####################################################

    def _move_to_start(self, limb='both'):
  
        starting_joint_angles_l = {'left_w0': 0.6699952259595108,
                                   'left_w1': 1.030009435085784,
                                   'left_w2': -0.4999997247485215,
                                   'left_e0': -1.189968899785275,
                                   'left_e1': 1.9400238130755056,
                                   'left_s0': -0.08000397926829805,
                                   'left_s1': -0.9999781166910306}
        starting_joint_angles_r = {'right_e0': -0.39888044530362166,
                                    'right_e1': 1.9341522973651006,
                                    'right_s0': 0.936293285623961,
                                    'right_s1': -0.9939970420424453,
                                    'right_w0': 0.27417171168213983,
                                    'right_w1': 0.8298780975195674,
                                    'right_w2': -0.5085333554167599}
        try:                
            if limb == 'left_gripper':
                if self._verbose:
                    print("Moving the left arm to start pose...")
                self._guarded_move_to_joint_position(limb, starting_joint_angles_l)
            elif limb == 'right_gripper':
                if self._verbose:
                    print("Moving the right arm to start pose...")
                self._guarded_move_to_joint_position(limb, starting_joint_angles_r)
            else:
                if self._verbose:
                    print("Moving the left arm to start pose...")
                self._guarded_move_to_joint_position('left_gripper', starting_joint_angles_l)
                if self._verbose:
                    print("Moving the right arm to start pose...")
                self._guarded_move_to_joint_position('right_gripper', starting_joint_angles_r)

            rospy.sleep(1.0)
            if self._verbose:
                print("At start position")
            return 1
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 0

    def _retract(self, gripperName):
        self._move_to_start(gripperName)

    def _approach(self, gripperName, pose, rate=100):
        appr = copy.deepcopy(pose)
        joint_angles = self.ik_request(gripperName, appr)
        self._guarded_move_to_joint_position(gripperName, joint_angles, rate)

    def _hover_approach(self, gripperName, pose):
        appr = copy.deepcopy(pose)
        appr.pose.position.z = appr.pose.position.z + self._hover_distance
        joint_angles = self.ik_request(gripperName, appr)
        self._guarded_move_to_joint_position(gripperName, joint_angles)

#####################################################################################################
######################### Internal Functions

    def _guarded_move_to_joint_position(self, limbName, joint_angles, rate=100):
        if joint_angles:
            limb = self.translateLimb(limbName)
            limb.move_to_joint_positions(positions=joint_angles, _rate=rate)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def translateGripper(self, gripperName):
        if 'left' in gripperName:
            return self._left_gripper
        else:
            return self._right_gripper

    def translateLimb(self, limbName):
        if  'left' in limbName:
            return self._left_limb
        else:
            return self._right_limb

    def translateIksvc(self, limbName):
        if  'left' in limbName:
            return self._iksvc_left
        else:
            return self._iksvc_right

    def ik_request(self, limbName, pose):
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(pose)
        try:
            iksvc = self.translateIksvc(limbName)
            resp = iksvc(ikreq)
            # print(resp)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 0
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return 0
        return limb_joints


####################################################################################################
