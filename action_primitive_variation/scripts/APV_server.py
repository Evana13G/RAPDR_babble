#!/usr/bin/env python

import rospy
import random

from action_primitive_variation.srv import *
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
        if T > 5: T = 5
        choices = paramDiscreteChoices[i_paramToVary].discretizedParamVals
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
    print('Action: ' + str(actionToVary) + ', Param: ' + str(paramToVary) + ', ' + str(paramAssignment))
    preconds = scenarioData().init
    paramActionExecutionProxy(actionToVary, args, [paramToVary], [str(paramAssignment)])
    effects = scenarioData().init
    novelty = novelEffectChecker(actionToVary, args, preconds, effects) 
    is_novel = novelty.novel_action
    new_effects = novelty.new_effects
    # print("New Effects: " + str(new_effects))
    return is_novel, new_effects

#### Call-back functions
def set_up_variations(req):

    #### Extract Info
    actionToVary = req.actionName
    args = req.args
    paramToVary = req.param
    T = req.T  
    env = req.environment

    if paramToVary == None or paramToVary == '':
        return APVSrvResponse(False)

    actionInfo = actionInfoProxy(actionToVary).actionInfo
    argNames = actionInfo.executableArgNames
    assert(len(argNames) == len(args))

    paramVals = process_intervals(actionInfo, paramToVary, T)

    for paramAssignment in paramVals:
        validity, new_effects = execute_and_evaluate_action(actionToVary, args, paramToVary, paramAssignment, env)
        if validity == True:
            newName = str(actionToVary) + '_' + str(paramToVary) + '_' + str(paramAssignment).split('.')[0]
            addActionToKB(actionToVary, newName, args, [paramToVary], [str(paramAssignment)], new_effects)
    return APVSrvResponse(validity)


def main():
    rospy.init_node("APV_node")
    rospy.wait_for_service('/raw_action_executor_srv')
    rospy.Service("APV_srv", APVSrv, set_up_variations)
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())
