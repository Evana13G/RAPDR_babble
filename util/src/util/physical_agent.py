#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np
import rospy
import rospkg
import time
import random
import os
import subprocess, signal
import moveit_commander
import moveit_msgs.msg
import tf

from math import pi
from moveit_commander.conversions import pose_to_list

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    GetModelState,
    GetLinkState,
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
    String
)

from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

from moveit_msgs.msg import (
    DisplayRobotState,
    RobotState,
    DisplayTrajectory,
    Grasp,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    VisibilityConstraint,
    GripperTranslation,
    JointLimits,
    LinkPadding,
)

##################################################################

class PhysicalAgent(object):
    def __init__(self, hover_distance = 0.15, verbose=False):
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._arm_group = moveit_commander.MoveGroupCommander("manipulator")
        self._grp_group = moveit_commander.MoveGroupCommander("gripper")
        # self._gripper_down_orientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)

####################################################################################################
############## Higher Level Action Primitives 

    def push(self, startPose, endPose):
        self._set_constraints(['base'])
        self.move_to_pose(startPose)
        self.gripper_close()
        self.move_to_pose(endPose)

    def grasp(self, pose):
        self._set_constraints(['base'])
        self.gripper_open()
        self.move_to_pose(pose)
        rospy.sleep(2)
        self.gripper_close(0.365) # Depends on the object 
        rospy.sleep(2)
        self.move_to_start()
        

    def shake(self, shakePose, twist_range, speed):
        return 1

    def press(self, objPose, hover_distance, press_amount):
        return 1

    def drop(self, objPose, drop_height):
        return 1


###################################################################################################
############## Lower Level Action Primitives 

    def gripper_open(self, position=0):
        self._move_gripper(position)
        if self._verbose:
            print("Gripper opened")
        return 1

    def gripper_close(self, position=0.8):
        self._move_gripper(position)
        if self._verbose:
            print("Gripper closed")
        return 1

    def move_to_start(self): 
        initial_joints = [1.6288508606798757e-05, -1.5999785607582009, -8.699362263797639e-05, -6.581909286129672e-05, -1.570796, 4.259259461747433e-05]
        self._arm_group.set_joint_value_target(initial_joints)
        self._arm_group.go(wait=True)
        print("Moved to start")
        if self._verbose:
            print("Moved to start")
        return 1

    def move_to_pose(self, pose):
        self._arm_group.set_pose_target(pose)
        self._arm_group.go(wait=True)

    def approach(self, pose):
        approachPose = copy.deepcopy(pose)
        hover_distance = 0.15
        approachPose.position.z = approachPose.position.z + hover_distance
        self._arm_group.set_pose_target(approachPose)
        self._arm_group.go(wait=True)

#####################################################################################################
######################### Internal Functions

    def _print_state(self):
        print("Robot State:")
        print(self._robot.get_current_state())

    def _move_gripper(self, value):
        # Value is from 0 to 1, where 0 is an open gripper, and 1 is a closed gripper        
        jointAngles = [(1*value), (-1*value), (1*value), (1*value), (-1*value), (1*value)]
        self._grp_group.set_joint_value_target(jointAngles)
        self._grp_group.go(wait=True)

    def _set_constraints(self, _constraints):
        self._disable_all_constraints()
        constraints = Constraints()
        constraints.name = "general_constraints"
        _joint_constraints = []
        if 'push' in _constraints:
            _joint_constraints = _joint_constraints + self._generate_push_constraints()
        if 'base' in _constraints:
            _joint_constraints = _joint_constraints + self._generate_base_constraints()

        constraints.joint_constraints = _joint_constraints
        self._arm_group.set_path_constraints(constraints)

    def _disable_all_constraints(self):
        self._arm_group.set_path_constraints(None)

    def _generate_base_constraints(self):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "shoulder_pan_joint"
        joint_constraint.position = 0
        joint_constraint.tolerance_above = 0.7854
        joint_constraint.tolerance_below = 0.7854
        joint_constraint.weight = 1

        return [joint_constraint]

    def _generate_push_constraints(self):
        constraints = []
        shoulder_constraint = JointConstraint()
        shoulder_constraint.joint_name = "shoulder_lift_joint"
        shoulder_constraint.position = -0.37754034809990955 #Obtained from looking at a good run in Gazebo
        shoulder_constraint.tolerance_above = 0.7853 #45 degrees
        shoulder_constraint.tolerance_below = 0.7853
        shoulder_constraint.weight = 1
        constraints.append(shoulder_constraint)

        elbow_constraint = JointConstraint()
        elbow_constraint.joint_name = "elbow_joint"
        elbow_constraint.position = 0.6780465480943496
        elbow_constraint.tolerance_above = 0.7853
        elbow_constraint.tolerance_below = 0.7853
        elbow_constraint.weight = 1
        constraints.append(elbow_constraint)

        # wrist_constraint = JointConstraint()
        # wrist_constraint.joint_name = "wrist_3_joint"
        # wrist_constraint.position = 0
        # wrist_constraint.tolerance_above = 0.1745
        # wrist_constraint.tolerance_below = 0.1745
        # wrist_constraint.weight = 1
        # constraints.append(wrist_constraint)
        
        return constraints
