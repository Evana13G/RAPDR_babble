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


########################################################################################################################
####                                                                             (is_novel, same_as_orig, new_effects)
####
#### ---- push, [rate]: 2.0 - 5.0                    **stays on table (False, True, []) 
#### ---- push, [rate]: 6.0 (ish) - 10.0             **knocks it off the table (True, True, ['(not (is_visible cover))'])
#### ---- push, [rate]: 11.0 - 150.0                 **stays on table (False, True, []) 
#### ----       tested: 11.0, 12.0, 25.0, 51.5, 100.0, 150.0
#### ----       default: 10.0 : knocks it off the table

#### ---- push, [movementMagnitude]: 0.1 - 0.26      **doesn't make contact (False, False, [])
#### ---- push, [movementMagnitude]: 0.27 - 0.38     **stays on table (False, True, []) 
#### ----       tested: 0.35, 0.36, 0.38, 0.3
#### ---- push, [movementMagnitude]: 0.39 - 0.61     **knocks it off the table (True, True, ['(not (is_visible cover))'])
#### ----       tested: 0.5, 0.4, 0.62 (False, True, [])
#### ---- push, [movementMagnitude]: 0.63 - 0.65     **doesn't make contact (False, False, [])
#### ---- push, [movementMagnitude]: 0.7 - ...       **breaks, not within moveable range  (False, False, [])
#### ----       default: 0.4 : knocks it off the table

#### --------- Maybe should change this to a theta value... 
#### ---- push, [orientation]: top                   **squeezes too hard, knocks it off the table (True, True, ['(not (is_visible cover))'])
#### ---- push, [orientation]: right                 **stays on table (False, True, []) 

#### --------- Need to test with cup and marbles... 
#### ---- shake, [rate]: 1.0                         **stays grasped, placed back down on its side (True, False (shaken), ['(not (covered cup))'])
#### ---- shake, [rate]: 10.5                        **stays grasped, placed back down on its side (True, False (shaken), ['(not (covered cup))'])
#### ---- shake, [rate]: 20.0                        **upright: stays grasped, placed back down upright (True, False (shaken), ['(not (covered cup))'])
#### ----        default: 15.0 : stays grasped, placed back down on its side 

#### ---- shake, [movementMagnitude]: 0.01           **stays grasped, placed back down on its side (True, False (shaken), ['(not (covered cup))'])
#### ---- shake, [movementMagnitude]: 2.5            **stays grasped, placed back down on its side (True, False (shaken), ['(not (covered cup))'])
#### ---- shake, [movementMagnitude]: 5.0            **stays grasped, placed back down on its side (True, False (shaken), ['(not (covered cup))'])
#### ----        default: 1.0 : stays grasped, placed back down on its side 

#### ---- shake, [orientation]: right
#### ---- shake, [orientation]: left
#### ----                  
        # POUR: 'orientation': 'left', 
        #       'rate' : '1.0'
        #       'movementMagnitude' '5.0'
        #

def test(req):
    print("--------------------------------------")
    print("----- TESTING PDDL CHECKER -----")

    try:
        # envProxy('restart', 'cook')
        envProxy('restart', 'cook_low_friction') 
        actionName = 'shake'
        args = ['left_gripper', 'cup']
        preconditions = scenarioData().init
        paramToVary = 'orientation'
        paramAssignment = 'left' 
        paramActionExecutionProxy(actionName, args, [paramToVary, 'rate', 'movementMagnitude'], [str(paramAssignment), '1.0', '5.0'])
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

