#!/usr/bin/env python


###### put a goal 
###### get a pddl plan for it 
###### try to execute it
###### if fails, get predicates list, and launch APV for each action until success
###### using the change points, test the actions and fire off the predicates list, storing all 
###### try each of the actions, return if successful
######analyze the new actions


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
from util.file_io import deleteAllPddlFiles,  deleteAllAPVFiles
import logging

import random
from std_msgs.msg import (
    Header,
    Empty,
)

def handle_trial(req):
    brainFilePath = os.path.dirname(os.path.realpath(__file__))
    resultsDir = generateResultsDir(brainFilePath, req.runName)

    logging.basicConfig(filename=resultsDir + 'output.log',level=logging.DEBUG)

    env = rospy.ServiceProxy('init_environment', HandleEnvironmentSrv)

    KB = KnowledgeBase()
    lPA = PhysicalAgent('left_gripper')
    rPA = PhysicalAgent('right_gripper')

    logging.info("\n################################################################################")
    logging.info("################################################################################")
    logging.info('## Action Primivitive Discovery in Robotic Agents through Action Segmentation ##')
    logging.info('## -- a proof of concept model for knowledge aquisition in intelligent agents ##')
    logging.info('## -- Evana Gizzi, Mateo Guaman Castro, Jivko Sinapov, 2018                   ##')
    logging.info("################################################################################")
    logging.info("################################################################################")
    try:
        # Services
        logging.info('\n ... Setting up services')
        planGenerator = rospy.ServiceProxy('plan_generator_srv', PlanGeneratorSrv)
        planExecutor = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)
        APV = rospy.ServiceProxy('APV_srv', APVSrv)
        partialActionExecutor = rospy.ServiceProxy('partial_plan_executor_srv', PartialPlanExecutorSrv)
        scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)

        logging.info(' ... Cleaning up data from last run')
        deleteAllPddlFiles()
        deleteAllAPVFiles()

        #####################################################################################
        #                                                                                   #
        #    This is where the highest level of the main algorithm should be implemented    #
        #                                                                                   #
        #    Domain:                     Problem:                                           #
        #    string name                 string task                                        #
        #    string[] requirements       string domain                                      #
        #    string[] types              string[] objects                                   #
        #    string[] predicates         string[] init                                      #
        #    string[] actions            string goal                                        #
        #                                                                                   #
        #####################################################################################
        logging.info(' ... Setting up initial data')
        task = 'APD'


        currentState = scenarioData()
        goalLoc = poseStampedToString(getPredicateLocation(currentState.predicateList.predicates, 'at', 'right_gripper'))
        goal1 = '(obtained block)'
        # goal2 = '(obj_at block ' + goalLoc  + ')'
        goal = [goal1]
        mode = ['diffsOnly', 'noLoc']
        newPrims = []
        gripperExecutingNewPrim = 'left'
        gripperExecutionValidity = True

        logging.info('\nAgent has the following goal: ')
        logging.info(goal)
        logging.info('\nAgent will attempt to accomplish this goal. If attempt fails, agent will try')
        logging.info('to find new actions and replan with those actions. Process repeats until the ')
        logging.info('agent is able to accomplish its goal....')
        attempt = 1
        attemptsTime = []
        totalTimeStart = time.time()
        while(goalAccomplished(goal, currentState.init) == False):
            trialStart = time.time()
            logging.info('\n***************************   ATTEMPT #' + str(attempt) + '   ***************************')
            logging.info('Setting up domain and problem for attempt #' + str(attempt))

            #####################################################################################
            domainDict = KB.getDomainData()
            domainName = domainDict['domain']
            types = domainDict['types']
            predicates = domainDict['predicates']
            requirements = domainDict['requirements']
            actions = domainDict['actions']
            # print(KB.getDomainData())
            logging.info(' -- Domain setup complete')

            #####################################################################################
            currentState = scenarioData()
            additionalLocations = domainDict['pddlLocs']
            
            initObjs = pddlObjects(currentState.predicateList.predicates, False)
            newPts = copy.deepcopy(initObjs['waypoint'])
            for loc in additionalLocations:
                newPts.append(loc)
            newPts.append(goalLoc)
            newPts = list(set(newPts))
            initObjs['waypoint'] = newPts
            objs = pddlObjectsStringFormat_fromDict(initObjs)
            init = currentState.init
            domain = Domain(domainName, requirements, types, predicates, actions)
            problem = Problem(task, domainName, objs, init, goal)
            filename = task + '_' + str(attempt)
            logging.info(' -- Problem setup complete')

            #####################################################################################
            logging.info('\nTriggering plan generation and execution for attempt #' + str(attempt))
            plan = planGenerator(domain, problem, filename, KB.getActionsLocs())
            logging.info(' -- Plan generation complete')
        
            # executionSuccess = planExecutor(plan.plan)
            # Just to gaurantee we go into APV mode for testing 

            if plan.plan.actions[0].params[0] == gripperExecutingNewPrim:
                gripperExecutionValidity = False
                executionSuccess = 0
            else:
                gripperExecutionValidity = True
                executionSuccess = planExecutor(plan.plan)

            #####################################################################################
            currentState = scenarioData()
            if (executionSuccess == 1):
                logging.info(' -- Plan execution complete')
                if (goalAccomplished(goal, currentState.init) == False):
                    logging.info(' ---- Goal COMPLETE ')
                    break
            else:
                logging.info(' -- Plan execution failed')
                moveLeftArmToStart(lPA)
                moveRightArmToStart(rPA)


                # Should prob break this into a diff module... findNewAction module 
                # Do one that can for each action, return new sub actoins to try ?

            #####################################################################################

            if newPrims == []:

                logging.info('\nGenerating all possible action/arg combinations (to send to APV) for attempt #' + str(attempt))
                momentOfFailurePreds = scenarioData().predicates
                APVtrials = []
                
                APVtrials.append(['obtain_object', 'left_gripper', 'wall', None]) 
                APVtrials.append(['obtain_object', 'left_gripper', 'table', None]) 
                APVtrials.append(['obtain_object', 'left_gripper', 'block', None]) 
                APVtrials.append(['press_button', 'left_gripper', 'left_button', None]) 
                APVtrials.append(['press_button', 'right_gripper', 'left_button', None]) 

                
                ##### BOTH need this #####
                #objectsToIterate = pddlObjects(currentState.predicateList.predicates, False)
                #for action in KB.getActions():#

                    ############ UNDER CONSTRUCTION ############
                #    args = action.getNonLocationVars()
                #    actionTrials = []
                #    actionTrial = []
                #    actionTrial.append(action.getName())
                #    actionTrial.append('left_gripper')
                #    actionTrials.append(actionTrial)
                #    lenTrials = len(actionTrials)
                #    newTrials = [] 
                #    for i in range(len(args)-1):
                #        for j in range(lenTrials):
                #            for argChoice in objectsToIterate[args[i+1]]:
                #                newTrial = copy.deepcopy(actionTrials[j])
                #                newTrial.append(argChoice)
                #                newTrials.append(newTrial)
                #        actionTrials = newTrials
                #    APVtrials.append(actionTrials)

                #addNones = copy.deepcopy(APVtrials)
                #replaceAPV = []
                #for trial in addNones:
                    #new = trial.append(None)
                #    replaceAPV.append(trial.append(None))
                #APVtrials = replaceAPV


                ############################################
                    
                logging.info(' -- generation complete, ' + str(len(APVtrials)) + ' total combos found')
                #for t in APVtrials:
                ###    print(t)

            #####################################################################################
                logging.info('\nFinding segmentation possibilities (across all combos generated) for attempt #' + str(attempt))
                trialNo = 0

                comboChoice = random.randint(0, len(APVtrials) - 1)
                logging.info("\n -- Combo # " + str(trialNo) + ': ' + str(APVtrials[comboChoice]))

                try:
                    #### Find change points    
                    resp = APV(APVtrials[comboChoice][0], APVtrials[comboChoice][1], APVtrials[comboChoice][2], APVtrials[comboChoice][3])
                    logging.info(' ---- ' + str(len(resp.endEffectorInfo)) + " total change points found")
                    logging.info("Trying partial plan execution on segmentations")
                    moveLeftArmToStart(lPA)
                    moveRightArmToStart(rPA)
                    #### Iterate across segmentations
                    i = 0
                    while i <= len(resp.endEffectorInfo) - 2:
                        # print(" ---- starting iteration #" + str(i+1))
                        startingState = scenarioData().predicateList
                        resp_2 = partialActionExecutor(APVtrials[comboChoice][1], resp.endEffectorInfo[i], resp.endEffectorInfo[i+1])
                        time.sleep(2)
                        endingState = scenarioData().predicateList

                        ##### Here is where you decide what gets added 
                        if(resp_2.success_bool == 1):
                            logging.info(' -- iteration ' + str(i) + ' successful!')

                            new_name = "action_attempt_" + str(attempt) + '_trial' + str(trialNo) + '_seg' + str(i) 
                                        #'.' + poseStampedToString(resp.endEffectorInfo[i]) + 
                                        #'.' + poseStampedToString(resp.endEffectorInfo[i+1])
                            orig_name = APVtrials[comboChoice][0]
                            orig_args = [APVtrials[comboChoice][1], APVtrials[comboChoice][2], APVtrials[comboChoice][3]]
                            gripperData = [resp.endEffectorInfo[i], resp.endEffectorInfo[i+1]]
                            gripper = orig_args[0]

                            newAction = KB.createAction(new_name, 
                                                        orig_name, 
                                                        orig_args,
                                                        startingState, 
                                                        endingState, 
                                                        PartialPlanExecutorSrv, 
                                                        gripper,
                                                        gripperData, 
                                                        mode)

                            if isViable(newAction):
                                logging.info(' ---- Segmentation VIABLE! Adding to knowledge base')
                                KB.addAction(newAction)
                                newPrims.append(newAction)
                        else:
                            logging.info(' -- iteration ' + str(i) + ' not successful')
                        i = i + 1 
                except rospy.ServiceException, e:
                    logging.info("Service call failed: %s"%e)
                del APVtrials[comboChoice]
                trialNo = trialNo + 1 
           
            else:
                if gripperExecutionValidity == True:
                    logging.info('Executing a Primitive')
                    if len(newPrims) == 1:
                        actionIndex = 0
                        actionToExecute = newPrims[actionIndex]
                    else:
                        actionIndex = random.randint(0, len(newPrims)-1)
                        actionToExecute = newPrims[actionIndex] 
                    resp_3 = partialActionExecutor(actionToExecute.getGripper(), actionToExecute.getExecutionParams()[0], actionToExecute.getExecutionParams()[1])
                    gripperExecutingNewPrim = actionToExecute.getGripper()
                    del newPrims[actionIndex]

            currentState = scenarioData()
            trialEnd = time.time()
            attemptsTime.append(trialEnd - trialStart)
            attempt = attempt + 1
        logging.info('\nGoal accomplished!')
        totalTimeEnd = time.time()
        logging.info("\nTimes for each trial (in s): ")
        logging.info(attemptsTime);
        logging.info("Total time elapsed: " + str(totalTimeEnd - totalTimeStart))

        compileResults(brainFilePath, req.runName)
        rospy.sleep(1)
        env('destroy')
        rospy.sleep(1)
        env('init')
        rospy.sleep(1)
    except rospy.ServiceException, e:
        logging.info("Service call failed: %s"%e)




def main():
    rospy.init_node("agent_brain_B")
    rospy.wait_for_message("/robot/sim/started", Empty)

    rospy.wait_for_service('APV_srv', timeout=60)
    rospy.wait_for_service('partial_plan_executor_srv', timeout=60)
    rospy.wait_for_service('scenario_data_srv', timeout=60)
    rospy.wait_for_service('plan_generator_srv', timeout=60)
    rospy.wait_for_service('plan_executor_srv', timeout=60)
    rospy.wait_for_service('press_button_srv', timeout=60)
    rospy.wait_for_service('obtain_object_srv', timeout=60)


    s = rospy.Service("brain_B_srv", BrainSrv, handle_trial)

    rospy.spin()

    return 0 

if __name__ == "__main__":
    main()
