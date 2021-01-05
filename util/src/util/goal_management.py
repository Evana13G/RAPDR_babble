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

def generateAllCombos_orig(T, scenario):
    APVtrials = []
    if scenario == 'discover_strike':
        APVtrials.append(['push', ['left_gripper', 'cover'], 'rate', T]) 
    elif scenario == 'discover_pour':
        APVtrials.append(['shake', ['left_gripper', 'cup'], 'orientation', T]) 
    elif scenario == 'cook':
        APVtrials.append(['push', ['left_gripper', 'cover'], 'rate', T]) 
        APVtrials.append(['push', ['left_gripper', 'cover'], 'orientation', T]) 
        APVtrials.append(['push', ['left_gripper', 'cover'], 'movementMagnitude', T]) 
        APVtrials.append(['shake', ['left_gripper', 'cup'], 'rate', T]) 
        APVtrials.append(['shake', ['left_gripper', 'cup'], 'orientation', T]) 
        APVtrials.append(['shake', ['left_gripper', 'cup'], 'movementMagnitude', T]) 
    return APVtrials  


############################################
## Prob need to move this to a service 
def generateAllCombos(T, plan, exploration_mode='focused'):
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
        if a.actionName not in ['uncover_obj', 'cover_obj', 'prep_food', 'cook', 'place_on_burner']:
            for param in ['rate', 'orientation', 'movementMagnitude']:
                formatted = []
                formatted.append(a.actionName)
                parsed_arg_vals = [x for x in a.argVals if '.' not in x] # Hack! Deadline in a week... send HALP
                formatted.append(parsed_arg_vals)
                formatted.append(param)
                formatted.append(T)
                formatted.append(exploration_mode)
                APVtrials.append(formatted)

    return APVtrials  
############################################