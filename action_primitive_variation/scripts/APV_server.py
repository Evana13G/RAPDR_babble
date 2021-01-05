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
        if T > 3: T = 3
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
    print('#### ---- ' + str(actionToVary) + ', [' + str(paramToVary) + ']: ' + str(paramAssignment))
    preconds = scenarioData().init
    paramActionExecutionProxy(actionToVary, args, [paramToVary], [str(paramAssignment)])
    effects = scenarioData().init
    
    novelty = novelEffectChecker(actionToVary, args, preconds, effects) 

    is_novel = novelty.novel_action
    # is_orig = novelty.same_effects_as_orig
    new_effects = novelty.new_effects
    return is_novel, new_effects

def set_up_variations(req):
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
            newName = str(actionToVary) + '-' + str(paramToVary) + ':' + str(paramAssignment).split('.')[0]
            addActionToKB(actionToVary, newName, args, [paramToVary], [str(paramAssignment)], new_effects)
    return APVSrvResponse(validity)

###################################################################################### 
# def generateAllCombos(req):
#     T = req.T
#     plan = req.plan

#     APVtrials = []
#     selections = []
#     mu = len(plan)
#     sd = 3.0

#     selection = -1.0
#     while len(plan) > 0:
#         selection = int(random.gauss(mu, sd))
#         if (0 <= selection < mu):
#             selections.append(plan[selection])
#             del plan[selection]
#             mu = len(plan) 

#     for a in selections:
#         formatted = []
#         formatted.append(a.actionName)
#         formatted.append(a.argVals)
#         formatted.append('rate')
#         formatted.append(T)
#         APVtrials.append(formatted)

#     # APVtrials.append(['push', ['left_gripper', 'cover'], 'rate', T]) 
#     return APVtrials  
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
