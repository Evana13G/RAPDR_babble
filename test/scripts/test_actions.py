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


def test_all_actions(req):
    print("---------------------------------------------------------------------------------------")
    print("---------------------------------   TESTING ACTIONS   ---------------------------------")

    T = 3 

    try:
        # DESIRED Args (and params) for ALL ACTIONS:
        # -- string gripper 
        # -- string objectName 
        # -- float64 movementMagnitude
        # -- float64 rate
        # -- string orientation


        #### PUSH ##########################################################
        print("Testing PUSH Scenario")
        print("----Original Environment: Cup and Cover")
        paramActionExecutionProxy('push', ['left_gripper', 'cover'], ['rate'], ['10.0'])
        
        envProxy('restart', 'HH')     
        print("----New Environment: Cup and HEAVY/HIGH FRICTION Cover")
        paramActionExecutionProxy('push', ['left_gripper', 'cover'], ['rate'], ['10.0'])

        envProxy('restart', 'HH')
        print("----New Environment: Increased Rate for PUSH")
        paramActionExecutionProxy('push', ['left_gripper', 'cover'], ['rate'], ['100.0'])

        envProxy('restart', 'default')    
        print("----Original Environment: Increased Rate for PUSH")
        paramActionExecutionProxy('push', ['left_gripper', 'cover'], ['rate'], ['100.0'])
        ####################################################################


        #### SHAKE #########################################################
        print("Testing SHAKE Scenario")
        envProxy('restart', 'default')
        print("----Original Environment: Cup and Cover")
        pddlActionExecutionProxy('shake', ['left_gripper', 'cup'])

        envProxy('restart', 'high_friction')
        print("----New Environment: Cup and HIGH FRICTION Cover")
        pddlActionExecutionProxy('shake', ['left_gripper', 'cup'])

        envProxy('restart', 'high_friction')
        print("----New Environment: Decreased Twist Range")
        paramActionExecutionProxy('shake', ['left_gripper', 'cup'], ['movementMagnitude'], ['3.0'])

        envProxy('restart', 'high_friction')
        print("----New Environment: Increased Twist Range")
        paramActionExecutionProxy('shake', ['left_gripper', 'cup'], ['movementMagnitude'], ['0.5'])

        envProxy('restart', 'high_friction')
        print("----New Environment: Decreased Rate")
        paramActionExecutionProxy('shake', ['left_gripper', 'cup'], ['rate'], ['0.1'])

        envProxy('restart', 'high_friction')
        print("----New Environment: Decreased Rate")
        paramActionExecutionProxy('shake', ['left_gripper', 'cup'], ['rate'], ['2.0'])

        ### Test newly found action on the original scenario
        ####################################################################

        #### PRESS #########################################################        
        # print("Testing PRESS Scenario")
        # envProxy('restart', 'default')

        ## COME BACK TO THIS

        # print("----Original Environment: Cup and Cover")
        # paramActionExecutionProxy('press', ['left_gripper', 'cover'], ['rate', 'movementMagnitude'], ['0.2', '0.1'])

        # envProxy('restart', 'heavy') 
        # pddlActionExecutionProxy('press', ['left_gripper', 'cover'])
        # envProxy('restart', 'default')
        
        # paramActionExecutionProxy('press', ['left_gripper', 'cover'], ['rate', 'movementMagnitude'], ['0.2', '500.0'])
        # envProxy('restart', 'default')  
        ####################################################################

        return True  
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return False

def main():
    rospy.init_node("test_actions")

    s = rospy.Service("test_actions_srv", EmptyTestSrv, test_all_actions)
    rospy.spin()

    return 0 


if __name__ == "__main__":
    main()


