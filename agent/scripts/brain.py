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

    print("\n#####################################################################################")
    print("#######################################################################################")
    print('## Action Primivitive Discovery in Robotic Agents through Action Parameter Variation ##')
    print('## -- a proof of concept model for knowledge aquisition in intelligent agents        ##')
    print('## -- Evana Gizzi, Amel Hassan, Jivko Sinapov, 2020                                  ##')
    print("#######################################################################################")
    print("#######################################################################################")


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
    gripperExecutingNewPrim = 'left'
    # gripperExecutionValidity = True

    try:
        print("\n\n##############################")
        print("#### -- Goal: " + str(goal))
        print("#### ---- Original Scenario (OS): ")
        success_bool = single_attempt_execution(task, goal, orig_env, additional_locs=additionalDomainLocs)
        print("#### ---- OS Execution Success: " + str(success_bool))
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

    return BrainSrvResponse([1], 1)

    try:
        print("#### ---- Novel Scenario (NS): ")
        attempt = 1
        totalTimeStart = rospy.get_time()
        currentState = scenarioData()

        while(goalAccomplished(goal, currentState.init) == False):
            
            trialStart = rospy.get_time()
            executionSuccess = single_attempt_execution(task, goal, orig_env, attempt, additionalDomainLocs)
            if (executionSuccess == 1):
                print(' -- Plan execution complete')
                currentState = scenarioData()
                if (goalAccomplished(goal, currentState.init) == True):
                    print(' ---- Goal COMPLETE ')
                    break
            else:
                print(' -- Plan execution failed')
                moveToStartProxy() #maybe this should be reset environment proxy...
            #####################################################################################

            if algoMode == 'APV':
                # TODO: Need to write an algo to do this intelligently 
                print('\nGenerating all possible action/arg combinations (to send to APV) for attempt #' + str(attempt))
                momentOfFailurePreds = scenarioData().predicates
                APVtrials = generateAllCombos(T)
                    
                print(' -- generation complete, ' + str(len(APVtrials)) + ' total combos found')
                for t in APVtrials:
                   print(t)

            #####################################################################################
                print('\nFinding segmentation possibilities (across all combos generated) for attempt #' + str(attempt))
                trialNo = 0

                while(len(APVtrials) >= 1): 
                # Pretty much remove this p for MODE 2 

                    # TODO Have this selective;; can probably encode it in the list in the function that generates 
                    # all combos 
                    comboChoice = random.randint(0, len(APVtrials) - 1)
                    comboToExecute = APVtrials[comboChoice]
                    comboToExecute.append(novel_env)

                    print("\n -- Combo # " + str(trialNo) + ': ' + str(comboToExecute))

                    try:

                        #### Find variations for this combo choice
                        resp = APVproxy(*comboToExecute)

                        ## I think APV should respond with actions 
                        ## to add to the knowledge base 

        #                 print(' ---- ' + str(len(resp.endEffectorInfo)) + " total change points found")
        #                 print("Trying partial plan execution on segmentations")
        #                 #### Iterate across segmentations
        #                 i = 0
        #                 while i <= len(resp.endEffectorInfo) - 2:
        #                     # print(" ---- starting iteration #" + str(i+1))
        #                     startingState = scenarioData().predicateList
        #                     resp_2 = partialActionExecutor(APVtrials[comboChoice][1], resp.endEffectorInfo[i], resp.endEffectorInfo[i+1])
        #                     time.sleep(2)
        #                     endingState = scenarioData().predicateList

        #                     ##### Here is where you decide what gets added 
        #                     if(resp_2.success_bool == 1):
        #                         print(' -- iteration ' + str(i) + ' successful!')

        #                         new_name = "action_attempt_" + str(attempt) + '_trial' + str(trialNo) + '_seg' + str(i) 
        #                                     #'.' + poseStampedToString(resp.endEffectorInfo[i]) + 
        #                                     #'.' + poseStampedToString(resp.endEffectorInfo[i+1])
        #                         orig_name = APVtrials[comboChoice][0]
        #                         orig_args = [APVtrials[comboChoice][1], APVtrials[comboChoice][2], APVtrials[comboChoice][3]]
        #                         gripperData = [resp.endEffectorInfo[i], resp.endEffectorInfo[i+1]]
        #                         gripper = orig_args[0]

        #                         newAction = KB.createAction(new_name, 
        #                                                     orig_name, 
        #                                                     orig_args,
        #                                                     startingState, 
        #                                                     endingState, 
        #                                                     PartialPlanExecutorSrv, 
        #                                                     gripper,
        #                                                     gripperData, 
        #                                                     mode)

        #                         if isViable(newAction):
        #                             print(' ---- Segmentation VIABLE! Adding to knowledge base')
        #                             KB.addAction(newAction)
        #                             newPrims.append(newAction)
        #                     else:
        #                         print(' -- iteration ' + str(i) + ' not successful')
        #                     i = i + 1 

                        print("continue here")
                        break
                    except rospy.ServiceException, e:
                        print("Service call failed: %s"%e)

                    del APVtrials[comboChoice]
                    trialNo = trialNo + 1 

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

def single_attempt_execution(task_name, goal, env, attempt='orig', additional_locs=[]):
    try:
        print("#### ------ [SINGLE ATTEMPT] ")
        envProxy('restart', env) if attempt != 'orig' else envProxy('no_action', env) 
        totalTimeStart = rospy.get_time()
        filename = task_name + '_' + str(attempt)

        initStateInfo = scenarioData()
        initObjsIncludingLoc = extendInitLocs(initStateInfo, additional_locs)
        objs = pddlObjectsStringFormat_fromDict(initObjsIncludingLoc)
        init = initStateInfo.init
        problem = Problem(task_name, KBDomainProxy().domain.name, objs, init, goal)
        plan = planGenerator(problem, filename)

        executionSuccess = planExecutor(plan.plan).success_bool

        if (executionSuccess == 1):
            print(' -- Plan execution complete')
            endStateInfo = scenarioData()

            if (goalAccomplished(goal, endStateInfo.init) == True):
                print(' ---- Goal COMPLETE ')
            return True
            
        else:
            print(' -- Plan execution failed')
            moveToStartProxy() #maybe this should be reset environment proxy...
            return False

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return False

def main():
    rospy.init_node("agent_brain")
    s = rospy.Service("brain_srv", BrainSrv, handle_trial)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()


