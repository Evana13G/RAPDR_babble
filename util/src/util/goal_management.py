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


# def findAllCombos(objects, action):
def isViable(action):
    if action.getEffects() == []:
        return False
    return True

def moveLeftArmToStart(lPa):
    starting_joint_angles_l = {'left_w0': 0.6699952259595108,
                               'left_w1': 1.030009435085784,
                               'left_w2': -0.4999997247485215,
                               'left_e0': -1.189968899785275,
                               'left_e1': 1.9400238130755056,
                               'left_s0': -0.08000397926829805,
                               'left_s1': -0.9999781166910306}
    lPa.move_to_start(starting_joint_angles_l)

def moveRightArmToStart(rPa):
    starting_joint_angles_r = {'right_e0': -0.39888044530362166,
                                'right_e1': 1.9341522973651006,
                                'right_s0': 0.936293285623961,
                                'right_s1': -0.9939970420424453,
                                'right_w0': 0.27417171168213983,
                                'right_w1': 0.8298780975195674,
                                'right_w2': -0.5085333554167599}
    rPa.move_to_start(starting_joint_angles_r)

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
    # APVtrials.append(['shake', ['left_gripper', 'cover'], 'speed']) 
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