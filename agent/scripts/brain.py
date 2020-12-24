#!/usr/bin/env python

import rospy
import time
import random
import multiprocessing

from agent.srv import *
from action_primitive_variation.srv import *
from environment.srv import *
from pddl.srv import *
from pddl.msg import *

from util.data_conversion import * 
from util.goal_management import *


APVproxy = rospy.ServiceProxy('APV_srv', APVSrv)
planGenerator = rospy.ServiceProxy('plan_generator_srv', PlanGeneratorSrv)
planExecutor = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
getObjLoc = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)
KBDomainProxy = rospy.ServiceProxy('get_KB_domain_srv', GetKBDomainSrv)
KBPddlLocsProxy = rospy.ServiceProxy('get_KB_pddl_locs', GetKBPddlLocsSrv)
envProxy = rospy.ServiceProxy('load_environment', HandleEnvironmentSrv)
moveToStartProxy = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
getScenarioSettings = rospy.ServiceProxy('scenario_settings_srv', GetScenarioSettingsSrv)


def handle_trial(req):

    print("\n########################################################")
    print("########################################################")
    print('##   Action Primivitive Discovery in Robotic Agents   ##') 
    print('##         through Action Parameter Variation         ##')
    print('##                                                    ##')
    print('## -- a proof of concept model for knowledge          ##')
    print('##    aquisition in intelligent agents                ##')
    print('##                                                    ##')
    print('## -- Evana Gizzi, Amel Hassan, Willy Lin,            ##')
    print('##    Keenan Rhea, Jivko Sinapov, 2020                ##')
    print('##                                                    ##')
    print("########################################################")
    print("########################################################\n")


    attemptsTime = []
    totalTimeStart = 0 ## TODO: Change this to SIM time. 
    task = req.runName
    scenario_settings = getScenarioSettings(req.scenario)
    goal = scenario_settings.goal
    orig_env = scenario_settings.orig_scenario
    novel_env = scenario_settings.novel_scenario
    T = scenario_settings.T
    additionalDomainLocs = scenario_settings.additional_domain_locs


    mode = ['diffsOnly', 'noLoc']
    algoMode = 'APV'
    newPrims = []
    gripperExecutingNewPrim = 'left_gripper'
    # gripperExecutionValidity = True

    currentState = scenarioData() # A bit of a hack for now
    
    try:
        print("################################################")
        print('#### ------------------------------------------ ')
        print("#### Goal: " + str(goal))
        print('#### ------------------------------------------ ')
        print("#### -- Original Scenario: ")
        # envProxy('restart', orig_env)
        outcome = single_attempt_execution(task, goal, orig_env, additional_locs=additionalDomainLocs)

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return BrainSrvResponse([1], 1)

    try:
        print("#### -- Novel Scenario: ")
        attempt = 0
        totalTimeStart = rospy.get_time()
        # currentState = scenarioData()
        action_exclusions = []

        while(goalAccomplished(goal, currentState.init) == False):
            
            # trialStart = rospy.get_time()
            outcome = single_attempt_execution(task, goal, novel_env, attempt, additionalDomainLocs, action_exclusions)

            if (outcome.goal_complete == True): break 
            currentState = scenarioData() #For the while loop (end on resetting this)
            action_exclusions.append(outcome.failure_action)

            #####################################################################################
            momentOfFailurePreds = scenarioData().predicates
            APVtrials = generateAllCombos(T, req.scenario)
            print("#### -- APV mode: " + str(len(APVtrials)) + " total combo(s) found")

            trialNo = 1

            while(len(APVtrials) >= 1): 
                comboChoice = random.randint(0, len(APVtrials) - 1)
                comboToExecute = APVtrials[comboChoice]
                comboToExecute.append(novel_env)
                # print("#### -- " + str(comboToExecute))
                resp = APVproxy(*comboToExecute)
                del APVtrials[comboChoice]
                trialNo = trialNo + 1 

            print("#### -- APV mode: COMPLETE")
            print('#### ------------------------------------------ ')
            attempt += 1

        #         # MODE 1: start
        #         algoMode = 'planAndRun'               
        #     else:
        #         if newPrims == []:
        #             print('No Prims to Execute')
        #             algoMode = 'APV' 
        #         else:
        #         # MODE 1: end
        #         # Pretty much, can remove this for MODE 2

        #             if gripperExecutionValidity == True:
        #                 print('Executing a Primitive')
        #                 if len(newPrims) == 1:
        #                     actionIndex = 0
        #                     actionToExecute = newPrims[actionIndex]
        #                 else:
        #                     actionIndex = random.randint(0, len(newPrims)-1)
        #                     actionToExecute = newPrims[actionIndex] 
        #                 resp_3 = partialActionExecutor(actionToExecute.getGripper(), actionToExecute.getExecutionParams()[0], actionToExecute.getExecutionParams()[1])
        #                 gripperExecutingNewPrim = actionToExecute.getGripper()
        #                 del newPrims[actionIndex]

        #     currentState = scenarioData()
        #     trialEnd = rospy.get_time()
        #     attemptsTime.append(trialEnd - trialStart)
        #     attempt = attempt + 1

        # print('\nGoal accomplished!')

        # totalTimeEnd = rospy.get_time()
        # print("\nTimes for each trial (in s): ")
        # print(str(attemptsTime))
        # print("Total time elapsed: " + str(totalTimeEnd - totalTimeStart))

        # compileResults(brainFilePath, req.runName)
        # processLogData(logFilePath, logData)        
        # return BrainSrvResponse(attemptsTime, totalTimeEnd - totalTimeStart) 
        return BrainSrvResponse([1], 1) # temp
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return BrainSrvResponse([1], 1) # temp
        # totalTimeEnd = rospy.get_time()
        # processLogData(logFilePath, logData)        
        # return BrainSrvResponse(attemptsTime, totalTimeEnd - totalTimeStart) 

    
#############################################################################
#############################################################################
#############################################################################
#############################################################################


def single_attempt_execution(task_name, goal, env, attempt='orig', additional_locs=[], action_exclusions=[]):
    
    # print('#### ------------------------------------------ ')
    if (attempt != 'orig' and attempt != 0):
        print("#### -- [ATTEMPT " + str(attempt)+ "] ") 
    filename = task_name + '_' + str(attempt)
    outcome = PlanExecutionOutcome(False, False, None)  

    try:
        envProxy('restart', env) if attempt != 'orig' else envProxy('no_action', env) 
    except rospy.ServiceException, e:
        print("Reset Environment Service call failed: %s"%e)
        return outcome

    try:
        totalTimeStart = rospy.get_time()
        initStateInfo = scenarioData()
        initObjsIncludingLoc = extendInitLocs(initStateInfo, additional_locs)
        initObjsIncludingLoc['gripper'] = ['left_gripper']
        initObjsIncludingLoc['obj'] = ['cup', 'cover']

        objs = pddlObjectsStringFormat_fromDict(initObjsIncludingLoc)
        init = initStateInfo.init
        init = [x for x in init if 'right_gripper' not in x]
        problem = Problem(task_name, KBDomainProxy().domain.name, objs, init, goal)
        plan = planGenerator(problem, filename, action_exclusions)
     
        if plan.plan.actions == []:
            print("#### ---- No Plan Found") 
            return outcome

        print("#### -- " + str([act.actionName for act in plan.plan.actions])) 
        # print(plan.plan.actions)
        outcome = planExecutor(plan.plan).execution_outcome
        endStateInfo = scenarioData().init
        outcome.goal_complete = goalAccomplished(goal, endStateInfo)

        print('#### ---- Plan execution: ' + str(outcome.execution_success))
        print('#### ---- Goal Accomplished: ' + str(outcome.goal_complete))
        print('#### ------------------------------------------ ')

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        moveToStartProxy()

    return outcome


def main():
    rospy.init_node("agent_brain")
    s = rospy.Service("brain_srv", BrainSrv, handle_trial)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()