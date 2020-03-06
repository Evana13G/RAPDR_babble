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
)

import tf
from tf.transformations import *

from environment.srv import * 
from agent.srv import MoveToStartSrv

pub = True

pub_cafe_table_pose = rospy.Publisher('cafe_table_pose', PoseStamped, queue_size = 10)
pub_block_pose = rospy.Publisher('block_pose', PoseStamped, queue_size = 10)
pub_left_gripper_pose = rospy.Publisher('left_gripper_pose', PoseStamped, queue_size = 10)
pub_right_gripper_pose = rospy.Publisher('right_gripper_pose', PoseStamped, queue_size = 10)
pub_cup_pose = rospy.Publisher('cup_pose', PoseStamped, queue_size = 10)
pub_cover_pose = rospy.Publisher('cover_pose', PoseStamped, queue_size = 10)

#SPAWN WALL AT 1.1525 z to be above table or 0.3755 to be below
def load_gazebo_models():

    table_pose=Pose(position=Point(x=0.78, y=0.0, z=0.0)),
    block_pose=Pose(position=Point(x=0.8, y=0.0185, z=0.8)),
    right_button_pose=Pose(position=Point(x=0.525, y=-0.2715, z=0.8)),
    left_button_pose=Pose(position=Point(x=0.525, y=0.1515, z=0.8)),
    block_reference_frame="world", 
    cup_pose=Pose(position=Point(x=0.5, y=0.0, z=0.9)),
    cover_pose=Pose(position=Point(x=0.5, y=0.0, z=0.9)),
    reference_frame="world"

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('environment')+"/models/"

    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    cup_xml = ''
    with open (model_path + "cup_with_cover/cup_model.sdf", "r") as cup_file:
        cup_xml=cup_file.read().replace('\n', '')
    cover_xml = ''
    with open (model_path + "cup_with_cover/cover_model.urdf", "r") as cover_file:
        cover_xml=cover_file.read().replace('\n', '')

    # Spawn Table SDF and other URDFs
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')

    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    global pub
    try:
        pub = False
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("cup")
        resp_delete = delete_model("cover")
    except rospy.ServiceException, e:
        pub = True
        rospy.loginfo("Delete Model service call failed: {0}".format(e))
        
def poseFromPoint(poseVar):
    newPose = poseVar.pose
    newPose.position.z -= 0.93
    # oldOrientationQ = newPose.orientation
    q_orientation = quaternion_from_euler(3.14, 0, 0).tolist()
    newPose.orientation = Quaternion(q_orientation [0], q_orientation [1], q_orientation [2],q_orientation [3])
    newPoseStamped = PoseStamped(header = poseVar.header, pose = newPose)
    return newPoseStamped


def init():

    SRVPROXY_move_to_start = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
    SRVPROXY_move_to_start()
    transform_listener = tf.TransformListener()

    load_gazebo_models()
    rospy.sleep(3)

    frameid_var = "/world"

    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/get_link_state')

    while pub == True:
        
        rate = rospy.Rate(10) # 10hz
        
        #Get cafe_table pose
        try:
            # (trans,rot) = transform_listener.lookupTransform('/world', '/base_link', rospy.Time(0))
            # print("TRANS: " + str(trans))
            # print("ROT: " + str(rot))

            cafe_table_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_cafe_table_ms = cafe_table_ms("cafe_table", "");
            pose_cafe_table = resp_cafe_table_ms.pose
            header_cafe_table = resp_cafe_table_ms.header
            header_cafe_table.frame_id = frameid_var
            poseStamped_cafe_table = PoseStamped(header=header_cafe_table, pose=pose_cafe_table)
            pub_cafe_table_pose.publish(poseFromPoint(poseStamped_cafe_table))

        except rospy.ServiceException, e:
            rospy.logerr("get_model_state for cafe_table service call failed: {0}".format(e))

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
        hdr = Header(frame_id=frameid_var)
        # hdr = Header(frame_id="")
        
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

        leftGripperPose = Pose()
        leftGripperPose.position.x = (pose_lglf.position.x + pose_lgrf.position.x)/2
        leftGripperPose.position.y = (pose_lglf.position.y + pose_lgrf.position.y)/2
        leftGripperPose.position.z = (pose_lglf.position.z + pose_lgrf.position.z)/2

        leftGripperPose.orientation = pose_lglf.orientation # TODO get the actual gripper orientation
        
        poseStamped_left_gripper = PoseStamped(header=hdr, pose=leftGripperPose)
        pub_left_gripper_pose.publish(poseStamped_left_gripper)

        pose_rglf = None
        pose_rgrf = None

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

        rightGripperPose = Pose()
        rightGripperPose.position.x = (pose_rglf.position.x + pose_rgrf.position.x)/2
        rightGripperPose.position.y = (pose_rglf.position.y + pose_rgrf.position.y)/2
        rightGripperPose.position.z = (pose_rglf.position.z + pose_rgrf.position.z)/2
        rightGripperPose.orientation = pose_rgrf.orientation # TODO get the actual gripper orientation
        
        poseStamped_right_gripper = PoseStamped(header=hdr, pose=rightGripperPose)
        pub_right_gripper_pose.publish(poseStamped_right_gripper)

     ####### END: Gripper pose processing
     ######################################################################################

def handle_environment_request(req):
    global pub 

    if req.action == "init":
        pub = True
        try:
            load_gazebo_models()
            return HandleEnvironmentSrvResponse(1)
        except rospy.ServiceException, e:
            rospy.logerr("Init environment call failed: {0}".format(e))
            return HandleEnvironmentSrvResponse(0)

    elif req.action == 'destroy':
        pub = False
        try:
            delete_gazebo_models()
            return HandleEnvironmentSrvResponse(1)
        except rospy.ServiceException, e:
            rospy.logerr("Destroy environment call failed: {0}".format(e))
            return HandleEnvironmentSrvResponse(0)

    elif req.action == 'restart':
        try:
            pub = False
            delete_gazebo_models()
            rospy.sleep(3)
            pub = True 
            load_gazebo_models()
            rospy.sleep(6)
            return HandleEnvironmentSrvResponse(1)

        except rospy.ServiceException, e:
            rospy.logerr("Destroy environment call failed: {0}".format(e))
            return HandleEnvironmentSrvResponse(0)
    else:
        print('No Action')


def main():

    rospy.init_node("initialize_environment_node")
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_service('move_to_start_srv', timeout=60)
    
    s = rospy.Service("init_environment", HandleEnvironmentSrv, handle_environment_request)
    init()
    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())

