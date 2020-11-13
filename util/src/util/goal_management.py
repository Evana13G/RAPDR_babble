from util.physical_agent import PhysicalAgent 
import os 

def goalAccomplished(goalList, currentState):
    numGoalsAccomplished = 0
    for goal in goalList:
        if goal in currentState:
            numGoalsAccomplished = numGoalsAccomplished + 1
    if numGoalsAccomplished == len(goalList):
        return True
    return False

def isViable(action):
    if action.getEffects() == []:
        return False
    return True

# def novel_effect(actual_preconds, actual_effects, expected_effects):
#     for pred in actual_effects:
#         if 'at' not in pred:
#             if (pred not in expected_effects) and (pred not in actual_preconds):
#                 return True
#     return False 

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


def generateAllCombos():
    APVtrials = []
    APVtrials.append(['push', ['left_gripper', 'cover'], 'rate']) 
    # APVtrials.append(['grasp', ['left_gripper', 'cover'], None]) 
    # APVtrials.append(['shake', ['left_gripper', 'cover'], 'twistRange']) 
    # APVtrials.append(['shake', ['left_gripper', 'cover'], 'rate']) 
    # APVtrials.append(['press', ['left_gripper', 'cover'], 'hoverDistance']) 
    # APVtrials.append(['press', ['left_gripper', 'cover'], 'pressAmount']) 
    # APVtrials.append(['press', ['left_gripper', 'cover'], 'rate']) 
    # APVtrials.append(['drop', ['left_gripper', 'cover'], 'dropHeight']) 


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