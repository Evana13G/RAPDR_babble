#!/usr/bin/env python

import rospy

from action_primitive_variation.srv import *
from pddl.srv import *
from agent.srv import * 
from environment.srv import *
from util.goal_management import *

actionInfoProxy = rospy.ServiceProxy('get_KB_action_info_srv', GetKBActionInfoSrv)
envResetProxy = rospy.ServiceProxy('load_environment', HandleEnvironmentSrv)
paramActionExecutionProxy = rospy.ServiceProxy('param_action_executor_srv', ParamActionExecutorSrv)
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
pddlInstatiations = rospy.ServiceProxy('get_pddl_instatiations_srv', GetActionPDDLBindingSrv)
addActionToKB = rospy.ServiceProxy('add_action_to_KB_srv', AddActionToKBSrv)

#### Access functions
def getObjectPose(object_name, pose_only=False):
    loc_pStamped = obj_location_srv(object_name)
    if pose_only == True:
        return loc_pStamped.location.pose
    return loc_pStamped.location

#### Helper functions
def process_intervals(actionInfo, paramToVary, T):
    paramNames = actionInfo.paramNames
    paramMins = list(actionInfo.paramMins)
    paramMaxs = list(actionInfo.paramMaxs)

    i_paramToVary = paramNames.index(paramToVary)
    paramMin = float(paramMins[i_paramToVary])
    paramMax = float(paramMaxs[i_paramToVary])
    I = (paramMax - paramMin)/(T-1)

    ## Process parameter values 
    paramVals = []
    for i in range(0, T-1):
        addition =  i * I
        paramVals.append(paramMin + addition)
    paramVals.append(paramMax)

    return paramVals

def detect_loc_changing_objects(actual_preconds, actual_effects, expected_effects):
    actual_preconds = [x for x in actual_preconds if 'at' in x]
    actual_effects = [x for x in actual_effects if 'at' in x]
    expected_effects = [x for x in expected_effects if 'at' in x]

    negativeLoc_objs = [x[9:].split()[0] for x in expected_effects if '(not ' in x]
    positiveLoc_objs = [x[4:].split()[0] for x in expected_effects if '(not ' not in x]

    expected_loc_changing_objects = [x for x in positiveLoc_objs if x in negativeLoc_objs]
    actual_loc_changing_objects = [pred[4:].split()[0] for pred in actual_effects if pred not in actual_preconds]

    print(expected_loc_changing_objects)
    print(actual_loc_changing_objects)

    if expected_loc_changing_objects == actual_loc_changing_objects:
        return []
    return actual_loc_changing_objects 

def novel_effect(actual_preconds, actual_effects, expected_effects):
    # locChange = detect_loc_changing_objects(actual_preconds, actual_effects, expected_effects)
    actual_preconds = [x for x in actual_preconds if 'at' not in x]
    actual_effects = [x for x in actual_effects if 'at' not in x]
    expected_effects = [x for x in expected_effects if 'at' not in x] 

    for pre in actual_preconds:
        if pre not in actual_effects:
            actual_effects.append('(not ' + pre + ')')

    actual_effects = [x for x in actual_effects if x not in actual_preconds]

    for pred in actual_effects:
        if pred not in expected_effects:
            # if locChange == []:
            return True, actual_effects

    return False, actual_effects

def execute_and_evaluate_action(actionToVary, args, paramToVary, paramAssignment, env):
    envResetProxy('restart', env)
    print('Action: ' + str(actionToVary) + ', Param: ' + str(paramToVary) + ', ' + str(paramAssignment))
    preconds = scenarioData().init
    paramActionExecutionProxy(actionToVary, args, [paramToVary], [str(paramAssignment)])
    effects = scenarioData().init
    expectation = pddlInstatiations(actionToVary, args).pddlBindings
    novelty, new_effects = novel_effect(preconds, effects, expectation.effects) 
    return novelty, new_effects

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
            newName = str(actionToVary) + '_' + str(paramToVary) + '_' + str(paramAssignment.split('.')[0])
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
