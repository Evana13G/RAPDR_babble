from util.physical_agent import PhysicalAgent 
import os 
import random 
import math 
import numpy as np

def goalAccomplished(goalList, currentState):
    numGoalsAccomplished = 0
    for goal in goalList:

        if goal in currentState:
            numGoalsAccomplished = numGoalsAccomplished + 1
        elif 'not' in goal:
            goal = goal[5:-1] # strip out the not operation (should abstract this)
            if goal not in currentState:
                numGoalsAccomplished = numGoalsAccomplished + 1
    if numGoalsAccomplished == len(goalList):
        return True
    return False

# def generateResultsDir(brainRunDirectory, resultsName):
#     resultsDir = brainRunDirectory + '/../../results/' + resultsName 
#     os.system('mkdir ' + resultsDir)
#     return resultsDir+'/'
    
def generateResultsDir(brainRunDirectory, resultsName):
    resultsDir = brainRunDirectory + '/../../results/' + resultsName 
    try:
        os.system('mkdir ' + resultsDir)
        return resultsDir+'/'
    except rospy.ServiceException, e:
        logData.append(("Unable to create results directory: %s"%e))


# def compileResults(brainRunDirectory, runName):
#     # outputFile = 'output.txt' 
#     resultsDir = brainRunDirectory + '/../../results/' + runName + '/'
#     pddlDir = brainRunDirectory + '/../../pddl/data/'
#     APVdir = brainRunDirectory + '/../../action_primitive_variation/data/'

#     # os.system('mv ' + outputFile + ' ' + resultsDir)
#     os.system('mv ' + pddlDir + '* ' + resultsDir)
#     os.system('mv ' + APVdir + '* ' + resultsDir)


def compileResults(brainRunDirectory, runName):
    resultsDir = brainRunDirectory + '/../../results/' + runName + '/'
    pddlDir = brainRunDirectory + '/../../pddl/data/'
    APVdir = brainRunDirectory + '/../../action_primitive_variation/data/'
    try:
        os.system('mv ' + pddlDir + '* ' + resultsDir)
        os.system('mv ' + APVdir + '* ' + resultsDir)
    except rospy.ServiceException, e:
        logData.append(("Unable to compile results: %s"%e))

def generateAllCombos(T, scenario):
    APVtrials = []
    if scenario == 'discover_strike':
        APVtrials.append(['push', ['left_gripper', 'cover'], 'rate', T]) 
    elif scenario == 'discover_pour':
        APVtrials.append(['shake', ['left_gripper', 'cup'], 'orientation', T]) 
    elif scenario == 'cook':
        APVtrials.append(['push', ['left_gripper', 'cover'], 'rate', T]) 
        APVtrials.append(['push', ['left_gripper', 'cover'], 'orientation', T]) 
        APVtrials.append(['shake', ['left_gripper', 'cup'], 'rate', T]) 
        APVtrials.append(['shake', ['left_gripper', 'cup'], 'orientation', T]) 
        APVtrials.append(['shake', ['left_gripper', 'cup'], 'movementMagnitude', T]) 

    return APVtrials  
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
## Prob need to move this to a service 
def generateAllCombos_dev(T, plan):
    APVtrials = []
    selections = []
    mu = len(plan)
    sd = 3.0

    selection = -1.0
    while len(plan) > 0:
        selection = int(random.gauss(mu, sd))
        if (0 <= selection < mu):
            selections.append(plan[selection])
            del plan[selection]
            mu = len(plan) 

    for a in selections:
        formatted = []
        formatted.append(a.actionName)
        formatted.append(a.argVals)
        formatted.append('rate')
        formatted.append(T)
        APVtrials.append(formatted)

# (['push', ['left_gripper', 'cover'], 'rate', T]) 
    # APVtrials.append(['push', ['left_gripper', 'cover'], 'rate', T]) 
    return APVtrials  
############################################