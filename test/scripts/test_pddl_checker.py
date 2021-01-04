#!/usr/bin/env python

import rospy

from agent.srv import *
from environment.srv import *
from pddl.srv import *

#### Service Proxies
novelEffect = rospy.ServiceProxy('novel_effect_srv', NovelEffectsSrv)
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
pddlActionExecutorProxy = rospy.ServiceProxy('pddl_action_executor_srv', PddlExecutorSrv)
paramActionExecutionProxy = rospy.ServiceProxy('param_action_executor_srv', ParamActionExecutorSrv)
envProxy = rospy.ServiceProxy('load_environment', HandleEnvironmentSrv)


#####################################################################################################
####                                                is_novel        same_as_orig        new_effects
####
#### ---- push, [rate]: 3.0                          False              True                []                  
#### ---- push, [rate]: 51.5                         False              True                []                           
#### ---- push, [rate]: 25.0                         False              True                []                           
#### ---- push, [rate]: 12.0                         False              True                []                           
#### ---- push, [rate]: 100.0, 150.0                 False              True                []  

#### ---- push, [movementMagnitude]: 0.1
#### ---- push, [movementMagnitude]: 0.35
#### ---- push, [movementMagnitude]: 0.6

#### ---- shake, [rate]: 1.0
#### ---- shake, [rate]: 10.5
#### ---- shake, [rate]: 20.0

#### ---- shake, [movementMagnitude]: 0.0
#### ---- shake, [movementMagnitude]: 2.5
#### ---- shake, [movementMagnitude]: 5.0

#### ---- push, [orientation]: left
#### ---- push, [orientation]: right
#### ---- push, [orientation]: front

#### ---- shake, [orientation]: top
#### ---- shake, [orientation]: right
#### ---- shake, [orientation]: left

def test(req):
    print("--------------------------------------")
    print("----- TESTING PDDL CHECKER -----")

    try:
        # envProxy('restart', 'cook')
        envProxy('restart', 'cook_low_friction') 
        actionName = 'push'
        args = ['left_gripper', 'cover']

        preconditions = scenarioData().init
        paramToVary = 'rate'
        paramAssignment = '5.0' 
        paramActionExecutionProxy(actionName, args, [paramToVary], [str(paramAssignment)])
        effects = scenarioData().init

        resp = novelEffect(actionName, args, preconditions, effects)
        
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

