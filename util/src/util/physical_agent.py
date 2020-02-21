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
        rospy.wait_for_service(ns_left, 5.0)
        rospy.wait_for_service(ns_right, 5.0)
        if self._verbose:
            print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        if self._verbose:
            print("Enabling robot... ")
        self._rs.enable()

####################################################################################################
############## Higher Level Action Primitives 

    def push(self, startPose, endPose):
        # self._set_constraints(['base'])
        # self.move_to_pose(startPose)
        # self.gripper_close()
        # self.move_to_pose(endPose)
        return 1

    def grasp(self, pose):
        # self._set_constraints(['base'])
        # self.gripper_open()
        # self.move_to_pose(pose)
        # rospy.sleep(2)
        # self.gripper_close(0.365) # Depends on the object 
        # rospy.sleep(2)
        # self.move_to_start()
        return 1

    def shake(self, shakePose, twist_range, speed):
        return 1

    def press(self, objPose, hover_distance, press_amount):
        return 1

    def drop(self, objPose, drop_height):
        return 1


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
    def _set_joint_velocity(self, start_angles=None, limb='both'):
        velocities_l = {'l_gripper_l_finger_joint' : 1.5129108885951033e-070, 
                        'l_gripper_r_finger_joint' : 2.169911245541326e-070,
                        'left_e0' : -1.7125872625407934e-060, 
                        'left_e1' : -7.172398941331635e-070,
                        'left_s0' : 1.2769639268640998e-070, 
                        'left_s1' : -1.9826054609577262e-080,
                        'left_w0' :  4.067311133343405e-050,
                        'left_w1' : 1.6401705346185865e-080,
                        'left_w2' : 5.840094167063631e-060}
        velocities_r = {'r_gripper_l_finger_joint' : 1.5129108885951033e-02, 
                'r_gripper_r_finger_joint' : 2.169911245541326e-02,
                'right_e0' : -1.7125872625407934e-02, 
                'right_e1' : -7.172398941331635e-02,
                'right_s0' : 1.2769639268640998e-02, 
                'right_s1' : -1.9826054609577262e-02,
                'right_w0' :  4.067311133343405e-02,
                'right_w1' : 1.6401705346185865e-02,
                'right_w2' : 5.840094167063631e-02}
        try:                
            if limb == 'left_gripper':
                if self._verbose:
                    print("Changing the left arm velocity ...")
                _limb = self.translateLimb(limb)
                _limb.set_joint_velocities(velocities_l)
            elif limb == 'right_gripper':
                if self._verbose:
                    print("Changing the right arm velocity ...")
                _limb = self.translateLimb(limb)
                _limb.set_joint_velocities(velocities_r)
            else:
                if self._verbose:
                    print("Changing the left arm velocity ...")
                _limb = self.translateLimb('left_gripper')
                _limb.set_joint_velocities(velocities_l)
                if self._verbose:
                    print("Changing the right arm velocity ...")
                _limb = self.translateLimb('right_gripper')
                _limb.set_joint_velocities(velocities_r)

            rospy.sleep(1.0)
            if self._verbose:
                print("Set joint velocities")
            return 1
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 0
    ####################################################

    def _move_to_start(self, start_angles=None, limb='both'):
  
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

    def move_to_pose(self, pose):
        self._arm_group.set_pose_target(pose)
        self._arm_group.go(wait=True)

    def _approach(self, gripperName, pose):
        appr = copy.deepcopy(pose)
        appr.pose.position.z = appr.pose.position.z + self._hover_distance
        joint_angles = self.ik_request(gripperName, appr)
        self._guarded_move_to_joint_position(gripperName, joint_angles)


#####################################################################################################
######################### Internal Functions

    def _guarded_move_to_joint_position(self, limbName, joint_angles):
        if joint_angles:
            limb = self.translateLimb(limbName)
            limb.move_to_joint_positions(joint_angles)
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
