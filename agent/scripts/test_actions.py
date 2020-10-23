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


pushProxy = rospy.ServiceProxy('push_srv', PushSrv)
shakeProxy = rospy.ServiceProxy('shake_srv', ShakeSrv)
pressProxy = rospy.ServiceProxy('press_srv', PressSrv)


def test_all_actions(req):
    print("---------------------------------------------------------------------------------------")
    print("---------------------------------   TESTING ACTIONS   ---------------------------------")

    T = 3 

    try:
        #### PUSH ##########################################################
        # Args:
        # -- string objectName 
        # -- float64 startOffset
        # -- float64 endOffset
        # -- int64 rate
        # 
        # print("----LOW mass, LOW friction, LOW velocity")
        # pushProxy('cover', None, None, 10.0)
        
        # envProxy('restart', 'HH')     
        # print("----HIGH mass, HIGH friction, LOW velocity")
        # pushProxy('cover', None, None, 10.0)

        # envProxy('restart', 'HH')
        # print("----HIGH mass, HIGH friction, HIGH velocity")
        # pushProxy('cover', None, None, 100.0)

        # envProxy('restart', 'default')    
        # print("----LOW mass, LOW friction, HIGH velocity")
        # pushProxy('cover', None, None, 100.0)
        ####################################################################


        #### SHAKE #########################################################
        # Args:
        # -- string objectName
        # -- float64 twistRange
        # -- float64 speed
        #
        # envProxy('restart', 'default')
        # print("----LOW mass, LOW friction")
        # shakeProxy('cup', None, None)

        # envProxy('restart', 'high_friction')
        # print("----Testing DEFAULT")
        # shakeProxy('cup', None, None)

        # envProxy('restart', 'high_friction')
        # print("----Testing twistRange: 3")
        # shakeProxy('cup', 3, None) 

        # envProxy('restart', 'high_friction')
        # print("----Testing twistRange: 0.5")
        # shakeProxy('cup', 0.5, None)


        # envProxy('restart', 'high_friction')
        # print("----Testing speed: 0.1")
        # shakeProxy('cup', None, 0.1)

        # envProxy('restart', 'high_friction')
        # print("----Testing speed: 2.0")
        # shakeProxy('cup', None, 2.0)
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

        return TestActionSrvResponse(True)  
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return TestActionSrvResponse(False) 

def main():
    rospy.init_node("test_actions")

    s = rospy.Service("test_actions_srv", EmptyTestSrv, test_all_actions)
    rospy.spin()

    return 0 


if __name__ == "__main__":
    main()


