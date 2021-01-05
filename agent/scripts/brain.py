#!/usr/bin/env python

import rospy
import time
import random

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
    scenario = req.scenario
    scenario_settings = getScenarioSettings(scenario)
    goal = scenario_settings.goal
    orig_env = scenario_settings.orig_scenario
    novel_env = scenario_settings.novel_scenario
    T = scenario_settings.T

    if scenario != 'discover_strike': envProxy('restart', orig_env)
    
    currentState = scenarioData() # A bit of a hack for now
    
    # try:
    #     print("################################################")
    #     print('#### ------------------------------------------ ')
    #     print("#### Goal: " + str(goal))
    #     print('#### ------------------------------------------ ')
    #     print("#### -- Original Scenario: ")
    #     outcome, _ = single_attempt_execution(task, goal, orig_env)

    # except rospy.ServiceException, e:
    #     print("Service call failed: %s"%e)
    #     return BrainSrvResponse([1], 1)

    try:
        print("#### -- Novel Scenario: ")

        # Global to Experiment. To be modified throughout experiment
        attempt = 0
        exploration_mode = 'focused'
        APVtrials = []
        action_exclusions = []
        trial_times = []
        exploration_times = []
        experiment_start = rospy.get_time()

        while(goalAccomplished(goal, currentState.init) == False):

            #####################################################################################
            ## Attempt
            ##
            # Timing Sequence
            trial_start = rospy.get_time()
            outcome, truncated_plan = single_attempt_execution(task, goal, novel_env, attempt, action_exclusions=action_exclusions)
            trial_end = rospy.get_time()
            trial_times.append(trial_end - trial_start)
            
            if (outcome.goal_complete == True): break 
            currentState = scenarioData() # post trial scenario, set it now. This is what you want evaluated
            #####################################################################################

            #####################################################################################
            ## Exploration
            ##
            ### Cleanup from attempt
            momentOfFailurePreds = scenarioData().predicates

            # init set or reset after exhausting. If its a reset, flip the exploration mode
            if APVtrials == []:
                if attempt > 0: exploration_mode = 'defocused'
                APVtrials = generateAllCombos(T, truncated_plan, exploration_mode)  
                print("#### -- APV mode: " + str(len(APVtrials)) + " total combo(s) found")

            comboChoice = random.randint(0, len(APVtrials) - 1)
            comboToExecute = APVtrials[comboChoice]
            just_cup_hack = ((novel_env in ['cook', 'cook_low_friction']) and (comboToExecute[0] == 'shake')) == True
            failure_env = 'just_cup' if (novel_env in ['cook', 'cook_low_friction']) else novel_env ## need to do this dynamically... maybe someday 
            comboToExecute.append(failure_env)
            
            # Timing Sequence
            exploration_start = rospy.get_time()
            resp = APVproxy(*comboToExecute)
            exploration_end = rospy.get_time()
            exploration_times.append(exploration_end - exploration_start)

            del APVtrials[comboChoice]

            # Setup for attempt
            action_exclusions = [comboToExecute[0]] # The one we are currently testing


            print("#### -- APV mode: COMPLETE")
            print('#### ------------------------------------------ ')
            attempt += 1


        experiment_end = rospy.get_time()
        total_experiment_time = experiment_end - experiment_start

        # compileResults(brainFilePath, req.runName)
        # processLogData(logFilePath, logData)        

        return BrainSrvResponse(exploration_times, total_experiment_time)
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return BrainSrvResponse([], 0)

    
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
        problem = Problem(task_name, KBDomainProxy(action_exclusions).domain.name, objs, init, goal)
        plan = planGenerator(problem, filename, action_exclusions)
     
        if plan.plan.actions == []:
            print("#### ---- No Plan Found ") 
            return outcome, []

        action_names = [act.actionName for act in plan.plan.actions]
        for a in action_names:
            print("#### ---- " + str(a))
        print("#### ---- ")

        outcome = planExecutor(plan.plan).execution_outcome
        endStateInfo = scenarioData().init
        outcome.goal_complete = goalAccomplished(goal, endStateInfo)

        truncated_plan = plan.plan.actions[:action_names.index(outcome.failure_action)]

        print('#### ---- Plan execution: ' + str(outcome.execution_success))
        print('#### ---- Goal Accomplished: ' + str(outcome.goal_complete))
        print('#### ------------------------------------------ ')

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        moveToStartProxy()

    return outcome, truncated_plan


def main():
    rospy.init_node("agent_brain")
    rospy.Service("brain_srv", BrainSrv, handle_trial)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()