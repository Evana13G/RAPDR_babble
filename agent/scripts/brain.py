#!/usr/bin/env python

import rospy
import time

from agent.srv import *
from action_primitive_variation.srv import *
from environment.srv import *
from pddl.srv import *
from pddl.msg import *
from util.knowledge_base import KnowledgeBase
from util.data_conversion import * 
from util.goal_management import *
from util.file_io import deleteAllPddlFiles, deleteAllAPVFiles, processLogData
import logging
import random
from std_msgs.msg import (
    Header,
    Empty,
)
from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)

pushProxy = rospy.ServiceProxy('push_srv', PushSrv)
graspProxy = rospy.ServiceProxy('grasp_srv', GraspSrv)
shakeProxy = rospy.ServiceProxy('shake_srv', ShakeSrv)
pressProxy = rospy.ServiceProxy('press_srv', PressSrv)
dropProxy = rospy.ServiceProxy('drop_srv', DropSrv)
# APVproxy = rospy.ServiceProxy('APV_srv', APVSrv)

KB = KnowledgeBase()
envProxy = rospy.ServiceProxy('load_environment', HandleEnvironmentSrv)

def handle_trial(req):
  
    # brainFilePath = os.path.dirname(os.path.realpath(__file__))
    # resultsDir = generateResultsDir(brainFilePath, req.runName)
    # logFilePath = resultsDir + 'output.txt'
    # logData =[]

    print("\n#####################################################################################")
    print("#######################################################################################")
    print('## Action Primivitive Discovery in Robotic Agents through Action Parameter Variation ##')
    print('## -- a proof of concept model for knowledge aquisition in intelligent agents        ##')
    print('## -- Evana Gizzi, Amel Hassan, Jivko Sinapov, 2020                                  ##')
    print("#######################################################################################")
    print("#######################################################################################")

    print("---------------------------------------------------------------------------------------")
    print("---------------------------------   TESTING ACTIONS   ---------------------------------")

    scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
    currentState = scenarioData()


    ####### APV testing Code #######
    # actionName = 'push'
    # args = ['left', 'cup', '0.1', '0.11']
    # param_to_vary = 'rate'
    # T = 3 # between 1 and 10?
    # APVproxy(actionName, args, param_to_vary, T)


    ##### Actions testing Code #####
  
    #### PUSH #######################################
    # pushProxy('cup', 0.1, 0.11, None) ## DEFAULT
    # envProxy('restart', 'heavy')      ## HEAVY    
    #
    # pushProxy('cup', 0.1, 0.11, None) ## DEFAULT
    # envProxy('restart', 'heavy')      ## HEAVY    
    #
    # pushProxy('cup', 0.1, 0.11, 500.0) ## HIGH RATE    
    #################################################


    #### SHAKE ######################################
    # shakeProxy('cup', None, None)     ## DEFAULT
    # envProxy('restart', 'heavy')      ## HEAVY    
    #
    # shakeProxy('cup', None, None)     ## DEFAULT
    # envProxy('restart', 'high_friction')      ## HEAVY    
    #
    shakeProxy('cup', 3, 0.5)     ## HIGH SPEED
    #################################################




    # print("---------------------------------------------------------------------------------------")
    # print("----------------------------------   RUNNING BRAIN   ----------------------------------")
    # attemptsTime = []
    # totalTimeStart = 0
    try:
    #     # Services
    #     print('\n ... Setting up services')
        
    #     planGenerator = rospy.ServiceProxy('plan_generator_srv', PlanGeneratorSrv)
    #     planExecutor = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)
    #     scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
    #     # APV = rospy.ServiceProxy('APV_srv', APVSrv)

    #     print(' ... Cleaning up data from last run')
    #     deleteAllPddlFiles()
    #     deleteAllAPVFiles()

    #     print(' ... Setting up initial data')
    #     task = 'APD'

    #     currentState = scenarioData()

        # goalLoc = poseStampedToString(getPredicateLocation(currentState.predicateList.predicates, 'at', 'right_gripper'))
        # goal1 = '(obtained block)'
        # goal = [goal1]
        # mode = ['diffsOnly', 'noLoc']
        # algoMode = 'APV'
        # newPrims = []
        # gripperExecutingNewPrim = 'left'
        # gripperExecutionValidity = True

        # print('\nAgent has the following goal: ')
        # print(str(goal))
        # print('\nAgent will attempt to accomplish this goal. If attempt fails, agent will try')
        # print('to find new actions and replan with those actions. Process repeats until the ')
        # print('agent is able to accomplish its goal....')
        # attempt = 1

        # totalTimeStart = rospy.get_time()

        # while(goalAccomplished(goal, currentState.init) == False):
        #     trialStart = rospy.get_time()
        #     print('\n***************************   ATTEMPT #' + str(attempt) + '   ***************************')
        #     print('Setting up domain and problem for attempt #' + str(attempt))

        #     #####################################################################################
        #     domainDict = KB.getDomainData()
        #     domainName = domainDict['domain']
        #     types = domainDict['types']
        #     predicates = domainDict['predicates']
        #     requirements = domainDict['requirements']
        #     actions = domainDict['actions']
        #     # print(KB.getDomainData())
        #     print(' -- Domain setup complete')

        #     #####################################################################################
        #     currentState = scenarioData()
        #     additionalLocations = domainDict['pddlLocs']
            
        #     initObjs = pddlObjects(currentState.predicateList.predicates, False)
        #     newPts = copy.deepcopy(initObjs['waypoint'])
        #     for loc in additionalLocations:
        #         newPts.append(loc)
        #     newPts.append(goalLoc)
        #     newPts = list(set(newPts))
        #     initObjs['waypoint'] = newPts
        #     objs = pddlObjectsStringFormat_fromDict(initObjs)
        #     init = currentState.init
        #     domain = Domain(domainName, requirements, types, predicates, actions)
        #     problem = Problem(task, domainName, objs, init, goal)
        #     filename = task + '_' + str(attempt)
        #     print(' -- Problem setup complete')

        #     #####################################################################################
        #     print('\nTriggering plan generation and execution for attempt #' + str(attempt))
        #     plan = planGenerator(domain, problem, filename, KB.getActionsLocs())
        #     print(' -- Plan generation complete')
        
        #     # executionSuccess = planExecutor(plan.plan)
        #     # Just to gaurantee we go into APV mode for testing 

        #     if plan.plan.actions[0].params[0] == gripperExecutingNewPrim:
        #         gripperExecutionValidity = False
        #         executionSuccess = 0
        #     else:
        #         gripperExecutionValidity = True
        #         executionSuccess = planExecutor(plan.plan)

        #     #####################################################################################
        #     currentState = scenarioData()
        #     if (executionSuccess == 1):
        #         print(' -- Plan execution complete')
        #         if (goalAccomplished(goal, currentState.init) == False):
        #             print(' ---- Goal COMPLETE ')
        #             break
        #     else:
        #         print(' -- Plan execution failed')
        #         moveLeftArmToStart(lPA)
        #         moveRightArmToStart(rPA)
        #     #####################################################################################

        #     # MODE 1: start
        #     if algoMode == 'APV':
        #     # MODE 1: end

        #         print('\nGenerating all possible action/arg combinations (to send to APV) for attempt #' + str(attempt))
        #         momentOfFailurePreds = scenarioData().predicates
        #         APVtrials = generateAllCombos()
                    
        #         print(' -- generation complete, ' + str(len(APVtrials)) + ' total combos found')
        #         #for t in APVtrials:
        #         ###    print(t)

        #     #####################################################################################
        #         print('\nFinding segmentation possibilities (across all combos generated) for attempt #' + str(attempt))
        #         trialNo = 0


        #         # MODE 1: start
        #         while(len(APVtrials) >= 1): 
        #         # MODE 1: end 
        #         # Pretty much remove this p for MODE 2 

        #             comboChoice = random.randint(0, len(APVtrials) - 1)
        #             print("\n -- Combo # " + str(trialNo) + ': ' + str(APVtrials[comboChoice]))

        #             try:
        #                 #### Find change points    
        #                 resp = APV(APVtrials[comboChoice][0], APVtrials[comboChoice][1], APVtrials[comboChoice][2], APVtrials[comboChoice][3], req.clusterThreshold, req.minClusterSize)
        #                 print(' ---- ' + str(len(resp.endEffectorInfo)) + " total change points found")
        #                 print("Trying partial plan execution on segmentations")
        #                 moveLeftArmToStart(lPA)
        #                 moveRightArmToStart(rPA)
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
        #             except rospy.ServiceException, e:
        #                 print("Service call failed: %s"%e)
        #             del APVtrials[comboChoice]
        #             trialNo = trialNo + 1 
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

def main():
    rospy.init_node("agent_brain")
    # rospy.wait_for_service('APV_srv', timeout=60)

    s = rospy.Service("brain_srv", BrainSrv, handle_trial)
    rospy.spin()

    return 0 


if __name__ == "__main__":
    main()


