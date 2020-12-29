#!/usr/bin/env python

# Modified from RethinkRobotics website

import argparse
import struct
import sys
import copy
import rospy
import rospkg
import random
import time


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

environment = 'default'
num_marbles = 30

pub_all = rospy.Publisher('models_loaded', Bool, queue_size=10)
moveToStartProxy = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
resetPreds = rospy.ServiceProxy('reset_env_preds', EmptySrvReq)

#SPAWN WALL AT 1.1525 z to be above table or 0.3755 to be below
def load_gazebo_models(env='default'):

    table_pose=Pose(position=Point(x=0.78, y=0.0, z=0.0))
    block_pose=Pose(position=Point(x=0.8, y=0.0185, z=0.8))
    right_button_pose=Pose(position=Point(x=0.525, y=-0.2715, z=0.8))
    left_button_pose=Pose(position=Point(x=0.525, y=0.1515, z=0.8))
    block_reference_frame="world"
    cup_pose=Pose(position=Point(x=0.5, y=0.0, z=0.9))
    cover_pose=Pose(position=Point(x=0.5, y=0.0, z=0.9))
    reference_frame="world"
    #####--Marbles--#####
    marbleB_pose=Pose(position=Point(x=0.0 + random.uniform(0.001,0.009), y=0.0 + random.uniform(0.001,0.009), z = 0.04))
    marbleR_pose=Pose(position=Point(x=0.0, y=0, z = 0.04))
    marble_reference_frame = "cup"


    # Get Models' Path
    model_path = rospkg.RosPack().get_path('environment')+"/models/"

    table_xml = ''
    cup_xml = ''
    cover_xml = ''
    marbleB_xml = ''
    marbleR_xml = ''


    moveToStartProxy('both')

    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')


    ###############################
    ############ HEAVY ############
    if env == 'heavy': 
        with open (model_path + "cup_with_cover/cup_model_heavy.sdf", "r") as cup_file:
            cup_xml=cup_file.read().replace('\n', '')
        with open (model_path + "cup_with_cover/cover_model_heavy.sdf", "r") as cover_file:
            cover_xml=cover_file.read().replace('\n', '')

    ###############################
    ######## HIGH FRICTION ########
    elif env == 'high_friction':
        with open (model_path + "cup_with_cover/cup_model_high_friction.sdf", "r") as cup_file:
            cup_xml=cup_file.read().replace('\n', '')
        with open (model_path + "cup_with_cover/cover_model_high_friction.sdf", "r") as cover_file:
            cover_xml=cover_file.read().replace('\n', '')


    ###############################
    ######## HEAVY HIGH FRICTION ##
    elif env == 'HH':
        with open (model_path + "cup_with_cover/cup_model.sdf", "r") as cup_file:
            cup_xml=cup_file.read().replace('\n', '')
        with open (model_path + "cup_with_cover/cover_model_heavy_high_friction.sdf", "r") as cover_file:
            cover_xml=cover_file.read().replace('\n', '')


    ###############################
    ########### DEFAULT ########### 
    else:
        with open (model_path + "cup_with_cover/cup_model_hollow.sdf", "r") as cup_file:
            cup_xml=cup_file.read().replace('\n', '')

        #####--Marbles--#####
        with open (model_path + "marbles/marbleB.sdf", "r") as marble_file:
            marbleB_xml=marble_file.read().replace('\n', '')
        with open (model_path + "marbles/marbleR.sdf", "r") as marble_file:
            marbleR_xml=marble_file.read().replace('\n', '')

        with open (model_path + "cup_with_cover/cover_model_high_friction.sdf", "r") as cover_file:
            cover_xml=cover_file.read().replace('\n', '')

    # Spawn Table SDF and other URDFs
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    try:

        spawn_sdf("cafe_table", table_xml, "/", table_pose, reference_frame)
        spawn_sdf("cup", cup_xml, "/", cup_pose, reference_frame)

        for i in range(num_marbles):
            resp_sdf = spawn_sdf("marbleB_"+ str(i), marbleB_xml, "/",
                                marbleB_pose, marble_reference_frame)
        rospy.sleep(2)

        for i in range(num_marbles):
            resp_sdf = spawn_sdf("marbleR_"+ str(i), marbleR_xml, "/",
                                marbleR_pose, marble_reference_frame)
        rospy.sleep(2)
        spawn_sdf("cover", cover_xml, "/", cover_pose, reference_frame)
        rospy.sleep(2)

    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    resetPreds()
    pub_all.publish(True)


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        pub_all.publish(False)
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        delete_model("cover")
        rospy.logerr("deleting cover")
        for i in range(num_marbles):
            delete_model("marbleR_"+ str(i))
            rospy.logerr("deleting marble_R"+ str(i))
        rospy.sleep(4)
        for i in range(num_marbles):
            delete_model("marbleB_"+ str(i))
            rospy.logerr("deleting marble_B" + str(i))
        rospy.sleep(4)
        delete_model("cup")
        resetPreds()
        # delete_model("cafe_table")

    except rospy.ServiceException as e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def handle_environment_request(req):
    action = req.action
    environment = 'default' if req.environment_setting == None else req.environment_setting
    if action == "init":
        try:
            rospy.sleep(1)
            load_gazebo_models(environment)
            rospy.sleep(3)
            return HandleEnvironmentSrvResponse(1)
        except rospy.ServiceException as e:
            rospy.logerr("Init environment call failed: {0}".format(e))
            return HandleEnvironmentSrvResponse(0)

    elif action == 'destroy':
        try:
            rospy.sleep(1)
            delete_gazebo_models()
            rospy.sleep(2)
            return HandleEnvironmentSrvResponse(1)
        except rospy.ServiceException as e:
            rospy.logerr("Destroy environment call failed: {0}".format(e))
            return HandleEnvironmentSrvResponse(0)

    elif action == 'restart':
        try:
            rospy.sleep(1)
            delete_gazebo_models()
            rospy.sleep(2)
            load_gazebo_models(environment)
            rospy.sleep(4)
            return HandleEnvironmentSrvResponse(1)

        except rospy.ServiceException as e:
            rospy.logerr("Destroy environment call failed: {0}".format(e))
            return HandleEnvironmentSrvResponse(0)
    else:
        print('No Action')
        return HandleEnvironmentSrvResponse(0)


def main():

    rospy.init_node("load_environment_node")
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_service('move_to_start_srv', timeout=60)
    rospy.wait_for_service('/gazebo/delete_model', timeout=60)
    
    s = rospy.Service("load_environment", HandleEnvironmentSrv, handle_environment_request)
    load_gazebo_models()

    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())

