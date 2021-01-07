from util.physical_agent import PhysicalAgent 
import os 
import random 
import math 
import numpy as np
from datetime import datetime
import csv

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

    
def generateExperimentDir(experimentRunDirectory, expName):
    resultsDir = experimentRunDirectory + '/../../results/'
    curr_dirs = [x[0].split('/')[-1] for x in os.walk(resultsDir)]
    if expName in curr_dirs:
        now = datetime.now()
        expName = expName + '_' + now.strftime("%d.%m.%Y_%H:%M:%S")
    expDir = resultsDir + expName 
    try:
        os.system('mkdir ' + expDir)
        return expDir +'/'
    except rospy.ServiceException, e:
        logData.append(("Unable to create experiment directory: %s"%e))
    
def generateRunResultsDir(experimentDirectory, runName, pddl_dir = False):
    curr_dirs = [x[0].split('/')[-1] for x in os.walk(experimentDirectory)]
    if runName in curr_dirs:
        now = datetime.now()
        runName = runName + '_' + now.strftime("%d.%m.%Y_%H:%M:%S")
    runResultsDir = experimentDirectory + runName 
    pddlResultsDir = runResultsDir + '/pddl/'
    try:
        os.system('mkdir ' + runResultsDir)
        if pddl_dir == True: os.system('mkdir ' + pddlResultsDir)
        return runResultsDir +'/'
    except rospy.ServiceException, e:
        logData.append(("Unable to create results directory: %s"%e))


def compileResults(experimentRunDir, runResultsDir):
    pddlDir = experimentRunDir + '/../../pddl/data/'
    try:
        os.system('cp ' + pddlDir + '* ' + runResultsDir + '/pddl/')

        existing_pddl_files = [x for x in os.listdir(pddlDir) if '.pddl' in x]
        if len(existing_pddl_files) > 0: os.system('rm ' + pddlDir + '*.pddl')  
        os.system('rm ' + runResultsDir  + '/pddl/description.txt')  
    except rospy.ServiceException, e:
        print("Unable to compile results: %s"%e)

def initResultCsvFile(result_filepath, result_name, csv_header):
    curr_files = [x for x in os.listdir(result_filepath) if '.csv' in x]
    result_file = result_filepath + result_name + '.csv'

    if result_file not in curr_files:
        with open(result_file, 'wb') as csvfile:
            filewriter = csv.writer(csvfile, delimiter=' ')
            filewriter.writerow(csv_header)
    else: 
        print('File already exists. Halting')
    return result_file

def writeResult(result_file, result):
    try: 
        with open(result_file, 'a') as csvfile:
            filewriter = csv.writer(csvfile, delimiter=' ')
            filewriter.writerow(result)
    except rospy.ServiceException, e:
        print("Unable to compile results: %s"%e)

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
            params = ['rate', 'orientation', 'movementMagnitude']
            while len(params) > 0:
                i = random.randint(0, len(params) - 1)
                param = params[i]
                formatted = []
                formatted.append(a.actionName)
                parsed_arg_vals = [x for x in a.argVals if '.' not in x] # Hack! Deadline in a week... send HALP
                formatted.append(parsed_arg_vals)
                formatted.append(param)
                formatted.append(T)
                formatted.append(exploration_mode)
                APVtrials.append(formatted)
                del params[i]

    return APVtrials  
############################################