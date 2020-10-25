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

APVproxy = rospy.ServiceProxy('APV_srv', APVSrv)
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
KBDomainProxy = rospy.ServiceProxy('get_KB_domain_srv', GetKBDomainSrv)
KBPddlLocsProxy = rospy.ServiceProxy('get_KB_pddl_locs', GetKBPddlLocsSrv)
envProxy = rospy.ServiceProxy('load_environment', HandleEnvironmentSrv)
moveToStartProxy = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
instatiatedPDDLBindings = rospy.ServiceProxy('get_pddl_instatiations_srv', GetActionPDDLBindingSrv)


pddlActionExecutionProxy = rospy.ServiceProxy('pddl_action_executor_srv', PddlExecutorSrv)
rawActionExecutionProxy = rospy.ServiceProxy('raw_action_executor_srv', RawActionExecutorSrv)
paramActionExecutionProxy = rospy.ServiceProxy('param_action_executor_srv', ParamActionExecutorSrv)


def test_action_settings(req):
    print("---------------------------------------------------------------------------------------")
    print("---------------------------   TESTING ACTION SETTINGS   -------------------------------")

    try:
        # To directly call actions on the physical agent executor, you need to specify 
        # all parameters. Therefore, I want to rework the actions so only the parameters 
        # listed in the chart can get set, and a 'NONE' setting is default. 

        #### PUSH ##########################################################
        # CURRENT Args (and params):        DESIRED Args (and params):
        # -- string objectName              -- string gripper 
        # -- float64 startOffset            -- string objectName 
        # -- float64 endOffset              -- float64 movementMagnitude
        # -- int64 rate                     -- float64 rate
        #                                   -- string orientation
        #
        # ORIGINAL ACTION CALL
        # pushProxy(objectName, startOffset, endOffset, rate)
        #
        # print("----Push Action")
        
        actionName = 'push'
        gripper = 'left_gripper'
        objectName = 'cover'
        args = [gripper, objectName]
        rate = '10.0'
        movementMagnitude = '0.4'
        orientation = 'left'
        controller = [rate, movementMagnitude, orientation]
        controllerNames = ['rate', 'movementMagnitude', 'orientation']
        # rawActionExecutionProxy(actionName, args, controller)
        paramActionExecutionProxy(actionName, args, controllerNames, controller)

        # envProxy('restart', 'default')     
        ####################################################################


        #### SHAKE #########################################################
        # CURRENT Args (and params):        DESIRED Args (and params):
        # -- string objectName              -- string gripper 
        # -- float64 twistRange             -- string objectName 
        # -- float64 speed                  -- float64 movementMagnitude
        #                                   -- float64 rate
        #                                   -- string orientation
        #
        # ORIGINAL ACTION CALL
        # shakeProxy(objectName, twistRange, speed)
        #
        # print("----Shake Action")
        # envProxy('restart', 'default')
        ####################################################################


        #### PRESS #########################################################
        # CURRENT Args (and params):        DESIRED Args (and params):
        # -- string objectName              -- string gripper 
        # -- float64 hoverDistance          -- string objectName 
        # -- float64 pressAmount            -- float64 movementMagnitude
        # -- float64 rate                   -- float64 rate
        #                                   -- string orientation
        #
        # ORIGINAL ACTION CALL
        # pressProxy(objectName, hoverDistance, pressAmount, rate)
        #
        # print("----Press Action")
        # envProxy('restart', 'default') 
        ####################################################################

        return True 
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return False 

def main():
    rospy.init_node("test_action_settings")
    rospy.Service("test_action_settings_srv", EmptyTestSrv, test_action_settings)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()


