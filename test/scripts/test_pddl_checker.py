#!/usr/bin/env python

import rospy

from agent.srv import *
from environment.srv import *
from pddl.srv import *
# from action_primitive_variation.srv import *

# from util.goal_management import *

#### Service Proxies

checkerProxy = rospy.ServiceProxy('check_effects_srv', CheckEffectsSrv)
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
pddlActionExecutorProxy = rospy.ServiceProxy('pddl_action_executor_srv', PddlExecutorSrv)


def test(req):
    print("--------------------------------------")
    print("----- TESTING PDDL CHECKER -----")

    try:
        actionName = 'push'
        args = ['left_gripper', 'cover']
        preconditions = scenarioData().init
        pddlActionExecutorProxy(actionName, args)
        effects = scenarioData().init

        resp = checkerProxy(actionName, args, preconditions, effects)
        
        return True 
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return False 

def main():
    rospy.init_node("test_pddl_checker_eval")
    rospy.Service("test_pddl_checker_srv", EmptyTestSrv, test)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()


