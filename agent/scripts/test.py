#!/usr/bin/env python

import rospy
import time

from agent.srv import *
from action_primitive_variation.srv import *
from environment.srv import *
from pddl.srv import *
from pddl.msg import *
from util.knowledge_base import KnowledgeBase
from util.data_conversion import * 
from util.goal_management import *
from util.file_io import deleteAllPddlFiles, deleteAllAPVFiles, processLogData
import logging
import random
from std_msgs.msg import (
    Header,
    Empty,
)
from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)

pushProxy = rospy.ServiceProxy('push_srv', PushSrv)
shakeProxy = rospy.ServiceProxy('shake_srv', ShakeSrv)
graspProxy = rospy.ServiceProxy('grasp_srv', GraspSrv)
pressProxy = rospy.ServiceProxy('press_srv', PressSrv)
dropProxy = rospy.ServiceProxy('drop_srv', DropSrv)

envProxy = rospy.ServiceProxy('load_environment', HandleEnvironmentSrv)

def handle_trial(req):

    print("\n#####################################################################################")
    print("#######################################################################################")
    print('## Action Primivitive Discovery in Robotic Agents through Action Parameter Variation ##')
    print('## -- a proof of concept model for knowledge aquisition in intelligent agents        ##')
    print('## -- Evana Gizzi, Amel Hassan, Jivko Sinapov, 2020                                  ##')
    print("#######################################################################################")
    print("#######################################################################################")

    print("---------------------------------------------------------------------------------------")
    print("---------------------------------   TESTING ACTIONS   ---------------------------------")

    scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
    currentState = scenarioData()

    ##### Actions testing Code #########################################
  
    #### PUSH ##########################################################
    # Args:
    # -- string objectName 
    # -- float64 startOffset
    # -- float64 endOffset
    # -- int64 rate
    #
    # pushProxy('cup', 0.1, 0.11, None)       ## DEFAULT
    # envProxy('restart', 'heavy')            ## HEAVY    
    #
    # pushProxy('cup', 0.1, 0.11, None)       ## DEFAULT
    # envProxy('restart', 'heavy')            ## HEAVY    
    #
    # pushProxy('cup', 0.1, 0.11, 500)        ## HIGH RATE 
    # envProxy('restart', 'default')          ## DEFAULT    
    ####################################################################


    #### SHAKE #########################################################
    # Args:
    # -- string objectName
    # -- float64 twistRange
    # -- float64 speed
    #
    # shakeProxy('cup', None, None)           ## DEFAULT
    # envProxy('restart', 'heavy')            ## HEAVY    
    #
    # shakeProxy('cup', None, None)           ## DEFAULT
    # envProxy('restart', 'high_friction')    ## HEAVY    
    #
    # shakeProxy('cup', 3, 0.1)               ## HIGH SPEED 
    #                                         ##(inverse relationship)
    # envProxy('restart', 'default')          ## DEFAULT  
    ####################################################################


    ## !!! NON VARIATION ACTION ########################################
    #### GRASP #########################################################
    # Args:
    # -- string objectName
    #
    # graspProxy('cup')                       ## NO VARIANTS
    # envProxy('restart', 'default')          ## DEFAULT  
    ####################################################################


    #### PRESS #########################################################
    # Args:
    # -- float64 hoverDistance
    # -- float64 pressAmount
    # -- float64 rate
    #
    # pressProxy('cup', None, None)           ## DEFAULT
    # envProxy('restart', 'heavy')            ## HEAVY    
    #
    # pressProxy('cup', None, None)           ## DEFAULT
    # envProxy('restart', 'high_friction')    ## HEAVY    
    #
    # pressProxy('cup', 3, 0.1)               ## HIGH SPEED 
    #                                         ##(inverse relationship)
    # envProxy('restart', 'default')          ## DEFAULT  
    ####################################################################

    

    return BrainSrvResponse([1], 1) 


def main():
    rospy.init_node("test_brain")

    s = rospy.Service("test_srv", BrainSrv, handle_trial)
    rospy.spin()

    return 0 


if __name__ == "__main__":
    main()


