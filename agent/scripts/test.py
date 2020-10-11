#!/usr/bin/env python

import rospy
import time

from agent.srv import *
from action_primitive_variation.srv import *
from environment.srv import *
from pddl.srv import *
from pddl.msg import *
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


APVproxy = rospy.ServiceProxy('APV_srv', APVSrv)
# planGenerator = rospy.ServiceProxy('plan_generator_srv', PlanGeneratorSrv)
# planExecutor = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)

KBDomainProxy = rospy.ServiceProxy('get_KB_domain_srv', GetKBDomainSrv)
KBPddlLocsProxy = rospy.ServiceProxy('get_KB_pddl_locs', GetKBPddlLocsSrv)
envProxy = rospy.ServiceProxy('load_environment', HandleEnvironmentSrv)
moveToStartProxy = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
instatiatedPDDLBindings = rospy.ServiceProxy('get_pddl_instatiations_srv', GetActionPDDLBindingSrv)

def handle_trial(req):

    print("---------------------------------------------------------------------------------------")
    print("---------------------------------   TESTING ACTIONS   ---------------------------------")

    APVtrials = generateAllCombos()

    trialNo = 0

    T = 3 
 
    comboChoice = random.randint(0, len(APVtrials) - 1)
    comboToExecute = APVtrials[comboChoice]
    comboToExecute.append(T)

    print("\n -- Combo # " + str(trialNo) + ': ' + str(comboToExecute))

    try:
        # currentState = scenarioData()
        actionStuff = instatiatedPDDLBindings('push', ['left_gripper', 'cover'])

        #### Find variations for this combo choice
        # resp = APVproxy(*comboToExecute)


    

    ##### Actions testing Code #########################################
  
    #### PUSH ##########################################################
    # Args:
    # -- string objectName 
    # -- float64 startOffset
    # -- float64 endOffset
    # -- int64 rate
    #
    # print("----LOW MASS, LOW VELOCITY")
    # pushProxy('cup', 0.05, 0.11, None)       ## DEFAULT
    # envProxy('restart', 'heavy')            ## HEAVY    
    # #
    # print("----HIGH MASS, LOW VELOCITY")
    # pushProxy('cup', 0.05, 0.11, None)       ## DEFAULT
    # envProxy('restart', 'heavy')            ## HEAVY    
    # #
    # print("----HIGH MASS, HIGH VELOCITY")
    # pushProxy('cup', 0.1, 0.11, 100000)        ## HIGH RATE 
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
    # -- string objectName
    # -- float64 hoverDistance
    # -- float64 pressAmount
    # -- float64 rate
    
    # pressProxy('cup', 0.1, 0.02, None)        ## DEFAULT
    # envProxy('restart', 'heavy')            ## HEAVY    
    #
    # pressProxy('cup', None, None, None)     ## DEFAULT
    # envProxy('restart', 'default')          ## DEFAULT    
    #
    # pressProxy('cup', None, 0.2, 500)       ## HIGH PRESS AMNT AND RATE 
    # envProxy('restart', 'default')            ## DEFAULT  
    ####################################################################


    #### DROP ##########################################################
    # Args:
    # -- string objectName
    # -- int64 dropHeight
    #
    # dropProxy('cup', 0.15)                  ## DEFAULT
    # envProxy('restart', 'heavy')            ## HEAVY    
    #
    # dropProxy('cup', 0.15)                  ## DEFAULT
    # envProxy('restart', 'default')          ## DEFAULT    
    #
    # dropProxy('cup', 0.1)                   ## HIGH DROP AMNT 
    # envProxy('restart', 'default')          ## DEFAULT  
    ####################################################################

        return BrainSrvResponse([1], 1) 
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return BrainSrvResponse([1], 1) # temp

def main():
    rospy.init_node("test_brain")

    s = rospy.Service("test_srv", BrainSrv, handle_trial)
    rospy.spin()

    return 0 


if __name__ == "__main__":
    main()


