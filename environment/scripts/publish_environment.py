#!/usr/bin/env python

# Modified from RethinkRobotics website

import argparse
import struct
import sys
import copy
import rospy
import rospkg
import random

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

pub_cafe_table_pose = rospy.Publisher('cafe_table_pose', PoseStamped, queue_size = 10)
pub_block_pose = rospy.Publisher('block_pose', PoseStamped, queue_size = 10)
pub_left_gripper_pose = rospy.Publisher('left_gripper_pose', PoseStamped, queue_size = 10)
pub_right_gripper_pose = rospy.Publisher('right_gripper_pose', PoseStamped, queue_size = 10)
pub_cup_pose = rospy.Publisher('cup_pose', PoseStamped, queue_size = 10)
pub_marble_pose = rospy.Publisher('marble_pose', PoseStamped, queue_size = 10)



<<<<<<< HEAD
def setPubAll(data):
    global pub_all
    pub_all = data.data

#SPAWN WALL AT 1.1525 z to be above table or 0.3755 to be below
def load_gazebo_models(table_pose=Pose(position=Point(x=0.78, y=0.0, z=0.0)),
                       block_pose=Pose(position=Point(x=0.8, y=0.0185, z=0.8)),
                       right_button_pose=Pose(position=Point(x=0.525, y=-0.2715, z=0.8)),
                       left_button_pose=Pose(position=Point(x=0.525, y=0.1515, z=0.8)),
                       block_reference_frame="world", 
                       cup_pose=Pose(position=Point(x=0.5, y=0.0, z=0.8)),
                       cover_pose=Pose(position=Point(x=0.5, y=0.0, z=0.9)),
                       marble_pose=Pose(position=Point(x=0.5, y=0.0 + random.uniform(0.1,0.3), z=0.9)),
                       reference_frame="world"):

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('environment')+"/models/"

    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    cup_xml = ''
    with open (model_path + "plastic_cup/model.sdf", "r") as cup_file:
        cup_xml=cup_file.read().replace('\n', '')

    marbleB_xml = ''
    with open (model_path + "marble/model.sdf", "r") as marble_file:
        marble_xml=marble_file.read().replace('\n', '')

    marbleR_xml = ''
    with open (model_path + "marble/modelR.sdf", "r") as marble_file:
        marble_xml=marble_file.read().replace('\n', '')

    # Spawn Table SDF and other URDFs
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')

    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("plastic_cup", cup_xml, "/",
                               cup_pose, reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
        
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        num_marbles = 5
        for i in range(num_marbles):
            resp_sdf = spawn_sdf("marbleB_"+ str(i), marble_xml, "/",
                                marble_pose, reference_frame)
        
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    global pub
    num_marbles = 5
    try:
        pub = False
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("plastic_cup")
        for i in range(num_marbles):         
            resp_delete = delete_model("marbleB_"+str(i))
    except rospy.ServiceException as e:
        pub = True
        rospy.loginfo("Delete Model service call failed: {0}".format(e))
=======
# def setPubAll(data):
#     global pub_all
#     pub_all = data.data
>>>>>>> 76f96f72725437586d99ae5a1f8560a3d162eb84
        
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
        cafe_table_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_cafe_table_ms = cafe_table_ms("cafe_table", "");
        pose_cafe_table = resp_cafe_table_ms.pose
        header_cafe_table = resp_cafe_table_ms.header
        header_cafe_table.frame_id = frameid_var
        poseStamped_cafe_table = PoseStamped(header=header_cafe_table, pose=pose_cafe_table)
        pub_cafe_table_pose.publish(poseFromPoint(poseStamped_cafe_table))

    except rospy.ServiceException, e:
        rospy.logerr("get_model_state for cafe_table service call failed: {0}".format(e))

<<<<<<< HEAD
    num_marbles = 5

    # print(pub_all)
    if pub_all == True:
        try:
            cafe_table_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_cafe_table_ms = cafe_table_ms("cafe_table", "");
            pose_cafe_table = resp_cafe_table_ms.pose
            header_cafe_table = resp_cafe_table_ms.header
            header_cafe_table.frame_id = frameid_var
            poseStamped_cafe_table = PoseStamped(header=header_cafe_table, pose=pose_cafe_table)
            pub_cafe_table_pose.publish(poseFromPoint(poseStamped_cafe_table))

        except rospy.ServiceException as e:
            rospy.logerr("get_model_state for cafe_table service call failed: {0}".format(e))

        try:
            marble_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for i in range(num_marbles):
                resp_marble_ms = marble_ms("marbleB_"+str(i), "")
            pose_marble = resp_marble_ms.pose
            header_marble = resp_marble_ms.header
            header_marble.frame_id = frameid_var
            poseStamped_marble = PoseStamped(header=header_marble, pose=pose_marble)
            pub_marble_pose.publish(poseFromPoint(poseStamped_marble))
        except rospy.ServiceException as e:
            rospy.logerr("get_model_state for block service call failed: {0}".format(e))

        try:
            cup_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_cup_ms = cup_ms("plastic_cup", "");
            pose_cup = resp_cup_ms.pose
            header_cup = resp_cup_ms.header
            header_cup.frame_id = frameid_var
            poseStamped_cup = PoseStamped(header=header_cup, pose=pose_cup)
            pub_cup_pose.publish(poseFromPoint(poseStamped_cup))
        except rospy.ServiceException as e:
            rospy.logerr("get_model_state for block service call failed: {0}".format(e))


     ######################################################################################
     ####### START: Gripper pose processing

        pose_lglf = None
        pose_lgrf = None
        hdr = Header(frame_id=frameid_var)
        # hdr = Header(frame_id="")
        
        try:
            lglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            resp_lglf_link_state = lglf_link_state('l_gripper_l_finger', 'world')
            # lglf_reference = resp_lglf_link_state.link_state.reference_frame
            pose_lglf = resp_lglf_link_state.link_state.pose
        except rospy.ServiceException as e:
            rospy.logerr("get_link_state for l_gripper_l_finger: {0}".format(e))
        try:
            lgrf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            resp_lgrf_link_state = lgrf_link_state('l_gripper_r_finger', 'world')
            # lgrf_reference = resp_lgrf_link_state.link_state.reference_frame
            pose_lgrf = resp_lgrf_link_state.link_state.pose
        except rospy.ServiceException as e:
            rospy.logerr("get_link_state for l_gripper_r_finger: {0}".format(e))
=======
    try:
        cover_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_cover_ms = cover_ms("cover", "");
        pose_cover = resp_cover_ms.pose
        header_cover = resp_cover_ms.header
        header_cover.frame_id = frameid_var
        poseStamped_cover = PoseStamped(header=header_cover, pose=pose_cover)
        pub_cover_pose.publish(poseFromPoint(poseStamped_cover))
    except rospy.ServiceException, e:
        rospy.logerr("get_model_state for block service call failed: {0}".format(e))
>>>>>>> 76f96f72725437586d99ae5a1f8560a3d162eb84

    try:
        cup_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_cup_ms = cup_ms("cup", "");
        pose_cup = resp_cup_ms.pose
        header_cup = resp_cup_ms.header
        header_cup.frame_id = frameid_var
        poseStamped_cup = PoseStamped(header=header_cup, pose=pose_cup)
        pub_cup_pose.publish(poseFromPoint(poseStamped_cup))
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
        lglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp_lglf_link_state = lglf_link_state('l_gripper_l_finger', 'world')
        # lglf_reference = resp_lglf_link_state.link_state.reference_frame
        pose_lglf = resp_lglf_link_state.link_state.pose
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for l_gripper_l_finger: {0}".format(e))
    try:
        lgrf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp_lgrf_link_state = lgrf_link_state('l_gripper_r_finger', 'world')
        # lgrf_reference = resp_lgrf_link_state.link_state.reference_frame
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

<<<<<<< HEAD
        pose_rglf = None
        pose_rgrf = None

        try:
            lglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            resp_rglf_link_state = lglf_link_state('r_gripper_l_finger', 'world')
            lglf_reference = resp_lglf_link_state.link_state.reference_frame
            pose_rglf = resp_rglf_link_state.link_state.pose
        except rospy.ServiceException as e:
            rospy.logerr("get_link_state for r_gripper_l_finger: {0}".format(e))
        try:
            rgrf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            resp_rgrf_link_state = rgrf_link_state('r_gripper_r_finger', 'world')
            # lgrf_reference = resp_lgrf_link_state.link_state.reference_frame
            pose_rgrf = resp_rgrf_link_state.link_state.pose
        except rospy.ServiceException as e:
            rospy.logerr("get_link_state for r_gripper_r_finger: {0}".format(e))
=======
    try:
        lglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp_rglf_link_state = lglf_link_state('r_gripper_l_finger', 'world')
        lglf_reference = resp_lglf_link_state.link_state.reference_frame
        pose_rglf = resp_rglf_link_state.link_state.pose
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for r_gripper_l_finger: {0}".format(e))
    try:
        rgrf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp_rgrf_link_state = rgrf_link_state('r_gripper_r_finger', 'world')
        # lgrf_reference = resp_lgrf_link_state.link_state.reference_frame
        pose_rgrf = resp_rgrf_link_state.link_state.pose
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for r_gripper_r_finger: {0}".format(e))
>>>>>>> 76f96f72725437586d99ae5a1f8560a3d162eb84

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

