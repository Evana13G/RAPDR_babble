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

####################################################################################################
############## Higher Level Action Primitives 

    def push(self, startPose, endPose, objPose):
        self._gripper_close("left")
        self._hover_approach("left", startPose)
        self._approach("left", startPose)
        self._approach("left", endPose)
        # efforts =  [0.0, -0.035200525640670714, -0.25935460745992633, 47.77774881151675, 23.51433066190589, 11.794216672654034, 23.60884891094095, -10.76398527560869, 15.0, -0.8971144799225961, 0.0, 0.0, -0.05173647335432463, -0.10614423477850465, 0.00023955770158679002, -0.23776845263334678, -0.009212185201725731, 0.020528554873848748, 0.0008220324551100333]
        self._approach('left', efforts)
        self._retract("left_gripper")
        return 1
        
    def push_effort(self, startPose, effort):
        self._gripper_close("left")
        self._hover_approach("left", startPose)
        self._approach("left", startPose)
        # self._apply_effort("left_s1", effort)
        
        body_name = 'left_wrist'
        ref_frame = ''
        ref_point = startPose.pose.position
        
        wrench = Wrench()
        force = Vector3()
        force.x = 0.0
        force.y = 0.0
        force.z = 0.0
        torque = Vector3()
        torque.x = 20.0
        torque.y = 0.0
        torque.z = 0.0
        wrench.force = force 
        wrench.torque = torque 

        start_time = rospy.Time(0.0) # start_time = rospy.Time.now()
        duration = rospy.Duration(5000.0)

        self._body_wrench_svc(body_name, ref_frame, ref_point, wrench, start_time, duration)
        # self._retract("left_gripper")
        rospy.sleep(5)
        return 1

    def grasp(self, pose):
        self._gripper_open("left")
        self._hover_approach("left", pose)
        self._approach("left", pose)
        return 1

    def shake(self, objPose, twist_range=1, speed=0.1):
        # For now, assume left gripper is moving (change to an argument)
        # Number of times to shake can be adjusted by the for loop 

        # twist_range tells you how much to move in each direction (delta)
        # speed has to do with the duration of the sleep between the two positions (frequency)


        lj = self._left_limb.joint_names()
        gripper_name = "left"

        joint_name= lj[6] # gripper twist, left_w2

        # GRIP OBJECT 
        self._gripper_open(gripper_name)
        self._hover_approach(gripper_name, objPose)
        self._approach(gripper_name, objPose)
        self._gripper_close(gripper_name)

        begin_position = self._left_limb.joint_angle(joint_name) # pose robot will move to at the end

        # Gripper moves in between two positions
        for i in range(100):
            current_position = self._left_limb.joint_angle(joint_name)
            joint_command = {joint_name: current_position + twist_range}
            self._left_limb.set_joint_positions(joint_command)
            time.sleep(speed)
            current_position = self._left_limb.joint_angle(joint_name)
            joint_command = {joint_name: current_position - twist_range}
            self._left_limb.set_joint_positions(joint_command)
            time.sleep(speed)
            
        joint_command = {joint_name: begin_position}
        self._left_limb.set_joint_positions(joint_command)

        return 1

    def press(self, objPose, hover_distance, press_amount): #TODO
        self._gripper_close("left")
        self._hover_approach("left", objPose)
        self._approach("left", objPose)
        return 1

    def drop(self, objPose, drop_height):
        self._gripper_open("left")
        self._hover_approach("left", objPose)
        self._approach("left", objPose)
        self._gripper_closed("left")
        self._hover_approach("left", objPose)
        self._gripper_open("left")
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
        # velocities_l = {'l_gripper_l_finger_joint' : 1.5129108885951033e-070, 
        #                 'l_gripper_r_finger_joint' : 2.169911245541326e-070,
        #                 'left_e0' : -1.7125872625407934e-060, 
        #                 'left_e1' : -7.172398941331635e-070,
        #                 'left_s0' : 1.2769639268640998e-070, 
        #                 'left_s1' : -1.9826054609577262e-080,
        #                 'left_w0' :  4.067311133343405e-050,
        #                 'left_w1' : 1.6401705346185865e-080,
        #                 'left_w2' : 5.840094167063631e-060}
        # velocities_r = {'r_gripper_l_finger_joint' : 1.5129108885951033e-02, 
        #         'r_gripper_r_finger_joint' : 2.169911245541326e-02,
        #         'right_e0' : -1.7125872625407934e-02, 
        #         'right_e1' : -7.172398941331635e-02,
        #         'right_s0' : 1.2769639268640998e-02, 
        #         'right_s1' : -1.9826054609577262e-02,
        #         'right_w0' :  4.067311133343405e-02,
        #         'right_w1' : 1.6401705346185865e-02,
        #         'right_w2' : 5.840094167063631e-02}


        velocities_l = {'l_gripper_l_finger_joint' : 1.5129108885951033e-070, 
                'l_gripper_r_finger_joint' : 2.169911245541326e-070,
                'left_e0' : -1.7125872625407934e-060, 
                'left_e1' : -7.172398941331635e-070,
                'left_s0' : 1.2769639268640998e-070, 
                'left_s1' : -1.9826054609577262e-080,
                'left_w0' :  4.067311133343405e-050,
                'left_w1' : 1.6401705346185865e-080,
                'left_w2' : 5.840094167063631e-060}
        velocities_r = {'r_gripper_l_finger_joint' : 2, 
                'r_gripper_r_finger_joint' : 2,
                'right_e0' : 12, 
                'right_e1' : 12,
                'right_s0' : 12, 
                'right_s1' : 12,
                'right_w0' : 12,
                'right_w1' : 12,
                'right_w2' : 12} 

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

    def _approach(self, gripperName, pose):
        appr = copy.deepcopy(pose)
        joint_angles = self.ik_request(gripperName, appr)
        self._guarded_move_to_joint_position(gripperName, joint_angles)

    def _hover_approach(self, gripperName, pose):
        appr = copy.deepcopy(pose)
        appr.pose.position.z = appr.pose.position.z + self._hover_distance
        joint_angles = self.ik_request(gripperName, appr)
        self._guarded_move_to_joint_position(gripperName, joint_angles)

#####################################################################################################
######################### Internal Functions

    # def _apply_effort(self, joint_name, effort):
    def _apply_effort(self, pose):
        efforts = []
        # self._approach(efforts)
        # print("START EFFORT")
        # start_time = rospy.Time(0.0)
        # start_time = rospy.Time.now()
        # duration = rospy.Duration(5000.0)
        # self._joint_effort_svc(joint_name, effort, start_time, duration)
        # rospy.sleep(5)
        # self._joint_effort_svc(joint_name, effort)
        print("END EFFORT")

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
