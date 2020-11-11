#!/usr/bin/env python

import rospy
import time

from agent.srv import *
from environment.srv import *
from pddl.srv import *

#### Service Proxies
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
pddlActionExecutionProxy = rospy.ServiceProxy('pddl_action_executor_srv', PddlExecutorSrv)
pddlInstatiations = rospy.ServiceProxy('get_pddl_instatiations_srv', GetActionPDDLBindingSrv)


def execute_and_evaluate_action(actionName, args):
    preconds = scenarioData().init  
    pddlActionExecutionProxy(actionName, args)
    effects = scenarioData().init
    expectation = pddlInstatiations(actionName, args).pddlBindings 
    # Possibly add params? Or ignore change of location

    nonLocEffect = novel_effect(effects, expectation.effects)
    if nonLocEffect:
        print(actionName + str(' is novel!'))

def test(req):
    print("--------------------------------------")
    print("----- TESTING PREDICATE SETTINGS -----")

    try:
        execute_and_evaluate_action('push', ['left_gripper', 'cover'])

        # envProxy('restart', 'default')
        return True 
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return False 

def main():
    rospy.init_node("test_predicate_check")
    rospy.Service("test_predicate_check_srv", EmptyTestSrv, test)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()


