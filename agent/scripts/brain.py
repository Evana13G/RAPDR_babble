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
KBDomainProxy = rospy.ServiceProxy('get_KB_domain_srv', GetKBDomainSrv)
KBPddlLocsProxy = rospy.ServiceProxy('get_KB_pddl_locs', GetKBPddlLocsSrv)
envProxy = rospy.ServiceProxy('load_environment', HandleEnvironmentSrv)
moveToStartProxy = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
getScenarioSettings = rospy.ServiceProxy('scenario_settings_srv', GetScenarioSettingsSrv)
getScenarioGoal = rospy.ServiceProxy('scenario_goal_srv', GetScenarioGoalSrv)

def handle_trial(req):

    task = req.runName
    scenario = req.scenario
    demo_mode = req.demo_mode
    scenario_settings = getScenarioSettings(scenario)
    orig_env = scenario_settings.orig_scenario
    novel_env = scenario_settings.novel_scenario
    T = scenario_settings.T

    envProxy('restart', orig_env)
    goal = getScenarioGoal(scenario).goal

    # Sim sensitive goals need to be re-calculated
    # if scenario in ['discover_strike']: goal = getScenarioSettings(scenario).goal  # Hack!
    
    currentState = scenarioData() # A bit of a hack for now
    
    print("################################################")
    print('#### ------------------------------------------ ')
    print("#### Goal: " + str(goal))
    print('#### ------------------------------------------ ')
    if demo_mode == True:
        try:
            print("#### -- Original Scenario: ")
            outcome, _ = single_attempt_execution(task, goal, orig_env)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
            return BrainSrvResponse([], 0)

    try:
        print("#### -- Novel Scenario: ")

        # Global to Experiment. To be modified throughout experiment
        attempt = 0
        exploration_mode = 'focused'
        APVtrials = []
        action_exclusions = []
        trial_times = []
        exploration_times = []

        # new_actions = []
        # new_actions_which_failed = []
        
        experiment_start = rospy.get_time()

        while(goalAccomplished(goal, currentState.init) == False):

            #####################################################################################
            ## Attempt
            ##
            # Timing Sequence
            trial_start = rospy.get_time()
            outcome, truncated_plan = single_attempt_execution(task, goal, novel_env, attempt, action_exclusions)
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
            # new_action_names = [act.actionName for act in truncated_plan]
            # new_action_names = [x for x in new_action_names if ':' in x]
            # new_actions_which_failed.extend(new_action_names)
            # if new_action_names != []:
            #     new_actions.remove(new_action_names)

            # init set or reset after exhausting. If its a reset, flip the exploration mode
            if APVtrials == []:
                if exploration_mode == 'defocused':
                    print("#### -- APV mode: COMPLETE")
                    print('#### ------------------------------------------ ')
                    print("#### -- None of the candidate actions worked! ")
                    print('#### ------------------------------------------ ')
                    break

                if attempt > 0: 
                    exploration_mode = 'defocused'
                APVtrials = generateAllCombos(T, truncated_plan, exploration_mode)  
                print("#### -- APV mode: " + str(len(APVtrials)) + " total combo(s) found")
            else:
                print("#### -- APV mode: START ")


            #####################################################################################
            # if new_actions == []:
            comboChoice = 0
            comboToExecute = APVtrials[comboChoice]

            just_cup_hack = ((novel_env in ['cook', 'cook_low_friction']) and (comboToExecute[0] == 'shake')) == True
            failure_env = novel_env  ## Need to do this dynamically... Maybe someday. RIP
            if just_cup_hack: failure_env = 'just_cup' 

            comboToExecute.append(failure_env)
        
            # Timing Sequence
            exploration_start = rospy.get_time()
            new_actions = APVproxy(*comboToExecute)
            exploration_end = rospy.get_time()
            exploration_times.append(exploration_end - exploration_start)

            del APVtrials[comboChoice]

            # Setup for attempt
            action_exclusions = [comboToExecute[0]] # The one we are currently testing
            #####################################################################################
        
            # action_exclusions.extend(new_actions_which_failed)

            print("#### -- APV mode: COMPLETE")
            print('#### ------------------------------------------ ')
            attempt += 1

        experiment_end = rospy.get_time()
        total_experiment_time = experiment_end - experiment_start

        # compileResults(brainFilePath, task)  

        return BrainSrvResponse(exploration_times, total_experiment_time)
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return BrainSrvResponse([], 0)

    
#############################################################################
#############################################################################
#############################################################################
#############################################################################


def single_attempt_execution(task_name, goal, env, attempt='orig', action_exclusions=[]):
    
    # print('#### ------------------------------------------ ')
    if (attempt != 'orig' and attempt != 0):
        print("#### -- [ATTEMPT " + str(attempt)+ "] ") 
    filename = task_name + '_' + str(attempt)
    outcome = PlanExecutionOutcome(False, False, None)  
    truncated_plan = []

    try:
        envProxy('restart', env) if attempt != 'orig' else envProxy('no_action', env) 
        rospy.sleep(1)
    except rospy.ServiceException, e:
        print("Reset Environment Service call failed: %s"%e)
        return outcome

    try:
        totalTimeStart = rospy.get_time()
        initStateInfo = scenarioData()
        initObjsIncludingLoc = extendInitLocs(initStateInfo, [])
        initObjsIncludingLoc['gripper'] = ['left_gripper']
        initObjsIncludingLoc['obj'] = ['cup', 'cover']

        objs = pddlObjectsStringFormat_fromDict(initObjsIncludingLoc)
        init = initStateInfo.init
        init = [x for x in init if 'right_gripper' not in x]
        problem = Problem(task_name, KBDomainProxy(action_exclusions).domain.name, objs, init, goal)

        plan = planGenerator(problem, filename, action_exclusions)
        action_list = plan.plan.actions

        if action_list == []:
            print("#### ---- No Plan Found ") 
            return outcome, truncated_plan

        action_names = [act.actionName for act in action_list]
        for a in action_names:
            print("#### ---- " + str(a))
        print("#### ---- ")

        outcome = planExecutor(plan.plan).execution_outcome
        endStateInfo = scenarioData().init
        outcome.goal_complete = goalAccomplished(goal, endStateInfo)

        if (outcome.failure_action != ''):
            if (outcome.failure_action is not None):
                af_index = action_names.index(outcome.failure_action) + 1
                truncated_plan = action_list[:af_index]  
        else:
            truncated_plan = action_list

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