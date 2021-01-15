#!/usr/bin/env python

import rospy
import random

from action_primitive_variation.srv import *
from action_primitive_variation.msg import *
from pddl.srv import *
from agent.srv import * 
from environment.srv import *
from util.goal_management import *

actionInfoProxy = rospy.ServiceProxy('get_KB_action_info_srv', GetKBActionInfoSrv)
envResetProxy = rospy.ServiceProxy('load_environment', HandleEnvironmentSrv)
paramActionExecutionProxy = rospy.ServiceProxy('param_action_executor_srv', ParamActionExecutorSrv)
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
addActionToKB = rospy.ServiceProxy('add_action_to_KB_srv', AddActionToKBSrv)
novelEffectChecker = rospy.ServiceProxy('novel_effect_srv', NovelEffectsSrv)

#### Access functions
def getObjectPose(object_name, pose_only=False):
    loc_pStamped = obj_location_srv(object_name)
    if pose_only == True:
        return loc_pStamped.location.pose
    return loc_pStamped.location

#### Helper functions
def process_intervals(actionInfo, paramToVary, T):
    paramNames= actionInfo.paramNames
    paramMins = list(actionInfo.paramMins)
    paramMaxs = list(actionInfo.paramMaxs)
    paramDiscreteChoices = actionInfo.discreteChoices

    i_paramToVary = paramNames.index(paramToVary)
    paramVals = []

    if paramToVary == 'orientation':
        if T > 3: T = 3
        choices = paramDiscreteChoices[i_paramToVary].discretizedParamVals#.remove('left') # remove = Hack 
        paramVals = random.sample(choices, k=T)
    else:
        paramMin = float(paramMins[i_paramToVary])
        paramMax = float(paramMaxs[i_paramToVary])
        I = (paramMax - paramMin)/(T-1)
        
        ## Process parameter values 
        for i in range(0, T-1):
            addition =  i * I
            paramVals.append(paramMin + addition)
        paramVals.append(paramMax)

    return paramVals

def execute_and_evaluate_action(actionToVary, args, paramToVary, paramAssignment, env):
    envResetProxy('restart', env)
    print('#### ---- ' + str(actionToVary) + ', [' + str(paramToVary) + ']: ' + str(paramAssignment))
    preconds = scenarioData().init
    exploration_start = rospy.get_time()
    paramActionExecutionProxy(actionToVary, args, [paramToVary], [str(paramAssignment)])
    exploration_end = rospy.get_time()
    
    exploration_time = exploration_end - exploration_start
    effects = scenarioData().init
    novelty = novelEffectChecker(actionToVary, args, preconds, effects) 

    is_novel = novelty.novel_action
    same_effects_as_orig = novelty.same_effects_as_orig
    new_effects = novelty.new_effects
    return is_novel, same_effects_as_orig, new_effects, exploration_time

def set_up_variations(req):
    actionToVary = req.actionName
    args = req.args
    paramToVary = req.param
    T = req.T  
    env = req.environment
    exploration_mode = req.exploration_mode

    exploration_time = 0.0
    variation_times = []

    if paramToVary == None or paramToVary == '':
        return APVSrvResponse(False, exploration_time, variation_times)

    actionInfo = actionInfoProxy(actionToVary).actionInfo
    argNames = actionInfo.executableArgNames

    print("DEBUGGING APV")
    print(actionToVary)
    print(argNames)
    print(args)
    assert(len(argNames) == len(args))

    paramVals = process_intervals(actionInfo, paramToVary, T)
    novel_actions = []

    for paramAssignment in paramVals:

        novel, accomplishes_OG_effects, new_effects, variation_time = execute_and_evaluate_action(actionToVary, args, paramToVary, paramAssignment, env)
        variation_times.append(variation_time)
        exploration_time += variation_time

        newName = str(actionToVary) + '-' + str(paramToVary) + ':' + str(paramAssignment)
 
        if accomplishes_OG_effects == True: 
            if exploration_mode == 'focused':
                if (novel == False): # Only add those which accomplish the same thing as the orig action
                    addActionToKB(actionToVary, newName, args, [paramToVary], [str(paramAssignment)], new_effects)
                    novel_actions.append(NovelAction(newName, args))

        elif exploration_mode == 'defocused':
            if (novel == True): # Add any actions which both: accomplish the same thing as the orig action, AND something novel
                addActionToKB(actionToVary, newName, args, [paramToVary], [str(paramAssignment)], new_effects)
                novel_actions.append(NovelAction(newName, args))
            # else: # If nothing specified, add it whether it is novel or not... this condition cant be reached in our research case
            #     addActionToKB(actionToVary, newName, args, [paramToVary], [str(paramAssignment)], new_effects)
            #     added_actions.append(newName)

    print('#### ---- ')
    print('#### ---- Newly added actions: ' + str([a.actionName for a in novel_actions]))
    return APVSrvResponse(novel_actions, exploration_time, variation_times)

###################################################################################### 


def main():
    rospy.init_node("APV_node")
    rospy.wait_for_service('/raw_action_executor_srv')
    rospy.Service("APV_srv", APVSrv, set_up_variations)
    # rospy.Service("generate_APV_combos", APVSrv, )
    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())
