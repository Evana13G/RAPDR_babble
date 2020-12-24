#!/usr/bin/env python

# Modified from RethinkRobotics website

import argparse
import struct
import sys
import copy
import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    GetModelState,
    GetLinkState,
)
from gazebo_msgs.msg import (
    LinkState,
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
    Bool,
)

import tf
from tf.transformations import *

from environment.srv import * 
from agent.srv import MoveToStartSrv

pub_all = None
environment = 'default'

getModelState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
getLinkState = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

pub_cafe_table_pose = rospy.Publisher('cafe_table_pose', PoseStamped, queue_size = 10)
pub_block_pose = rospy.Publisher('block_pose', PoseStamped, queue_size = 10)
pub_left_gripper_pose = rospy.Publisher('left_gripper_pose', PoseStamped, queue_size = 10)
pub_right_gripper_pose = rospy.Publisher('right_gripper_pose', PoseStamped, queue_size = 10)
pub_cup_pose = rospy.Publisher('cup_pose', PoseStamped, queue_size = 10)
pub_cover_pose = rospy.Publisher('cover_pose', PoseStamped, queue_size = 10)
pub_burner_pose = rospy.Publisher('burner1_pose', PoseStamped, queue_size = 10)

        
def poseFromPoint(poseVar):
    newPose = poseVar.pose
    newPose.position.z -= 0.93
    q_orientation = quaternion_from_euler(3.14, 0, 0).tolist()
    newPose.orientation = Quaternion(q_orientation [0], q_orientation [1], q_orientation [2],q_orientation [3])
    newPoseStamped = PoseStamped(header = poseVar.header, pose = newPose)
    return newPoseStamped

def publish(environment='default'):
    # rospy.wait_for_message("/models_loaded", Bool) 
    
    frameid_var = "/world"

    try:
        resp_cafe_table_ms = getModelState("cafe_table", "");
        pose_cafe_table = resp_cafe_table_ms.pose
        # pose_cafe_table.position.z += 0.7
        header_cafe_table = resp_cafe_table_ms.header
        header_cafe_table.frame_id = frameid_var
        poseStamped_cafe_table = PoseStamped(header=header_cafe_table, pose=pose_cafe_table)
        pub_cafe_table_pose.publish(poseStamped_cafe_table)

    except rospy.ServiceException, e:
        rospy.logerr("get_model_state for cafe_table service call failed: {0}".format(e))

    try:
        resp_cover_ms = getModelState("cover", "");
        pose_cover = resp_cover_ms.pose
        header_cover = resp_cover_ms.header
        header_cover.frame_id = frameid_var
        poseStamped_cover = PoseStamped(header=header_cover, pose=pose_cover)
        pub_cover_pose.publish(poseFromPoint(poseStamped_cover))
    except rospy.ServiceException, e:
        rospy.logerr("get_model_state for block service call failed: {0}".format(e))

    try:
        resp_cup_ms = getModelState("cup", "");
        pose_cup = resp_cup_ms.pose
        header_cup = resp_cup_ms.header
        header_cup.frame_id = frameid_var
        poseStamped_cup = PoseStamped(header=header_cup, pose=pose_cup)
        pub_cup_pose.publish(poseFromPoint(poseStamped_cup))
    except rospy.ServiceException, e:
        rospy.logerr("get_model_state for block service call failed: {0}".format(e))


    try:
        resp_burner_ms = getModelState("burner1", "");
        pose_burner = resp_burner_ms.pose
        header_burner = resp_burner_ms.header
        header_burner.frame_id = frameid_var
        poseStamped_burner = PoseStamped(header=header_burner, pose=pose_burner)
        pub_burner_pose.publish(poseFromPoint(poseStamped_burner))
    except rospy.ServiceException, e:
        rospy.logerr("get_model_state for block service call failed: {0}".format(e))


 ######################################################################################
 ####### START: Gripper pose processing

    pose_lglf = None
    pose_lgrf = None
    pose_rglf = None
    pose_rgrf = None

    hdr = Header(frame_id=frameid_var)

    try:
        resp_lglf_link_state = getLinkState('l_gripper_l_finger', 'world')
        pose_lglf = resp_lglf_link_state.link_state.pose
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for l_gripper_l_finger: {0}".format(e))
    try:
        resp_lgrf_link_state = getLinkState('l_gripper_r_finger', 'world')
        pose_lgrf = resp_lgrf_link_state.link_state.pose
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for l_gripper_r_finger: {0}".format(e))

    try:
        leftGripperPose = Pose()
        leftGripperPose.position.x = (pose_lglf.position.x + pose_lgrf.position.x)/2
        leftGripperPose.position.y = (pose_lglf.position.y + pose_lgrf.position.y)/2
        leftGripperPose.position.z = (pose_lglf.position.z + pose_lgrf.position.z)/2

        leftGripperPose.orientation = pose_lglf.orientation # TODO get the actual gripper orientation
        
        poseStamped_left_gripper = PoseStamped(header=hdr, pose=leftGripperPose)
        pub_left_gripper_pose.publish(poseStamped_left_gripper)
    except rospy.ServiceException, e:
        rospy.logerr("Unable to calculate calibrated position: {0}".format(e))

    try:
        resp_rglf_link_state = getLinkState('r_gripper_l_finger', 'world')
        lglf_reference = resp_lglf_link_state.link_state.reference_frame
        pose_rglf = resp_rglf_link_state.link_state.pose
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for r_gripper_l_finger: {0}".format(e))
    try:
        resp_rgrf_link_state = getLinkState('r_gripper_r_finger', 'world')
        pose_rgrf = resp_rgrf_link_state.link_state.pose
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for r_gripper_r_finger: {0}".format(e))

    try:
        rightGripperPose = Pose()
        rightGripperPose.position.x = (pose_rglf.position.x + pose_rgrf.position.x)/2
        rightGripperPose.position.y = (pose_rglf.position.y + pose_rgrf.position.y)/2
        rightGripperPose.position.z = (pose_rglf.position.z + pose_rgrf.position.z)/2
        rightGripperPose.orientation = pose_rgrf.orientation # TODO get the actual gripper orientation
    
        poseStamped_right_gripper = PoseStamped(header=hdr, pose=rightGripperPose)
        pub_right_gripper_pose.publish(poseStamped_right_gripper)
    except rospy.ServiceException, e:
        rospy.logerr("Unable to calculate calibrated position: {0}".format(e))


####### END: Gripper pose processing
######################################################################################


def main():

    rospy.init_node("publish_environment_node")
    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/get_link_state')

    rate = rospy.Rate(10) # 10hz

    rospy.wait_for_message("/models_loaded", Bool)
    
    while not rospy.is_shutdown():
        publish()
        rate.sleep()

if __name__ == '__main__':
    sys.exit(main())

