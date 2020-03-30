
#!/usr/bin/env python

import sys
import cv2
import math
import time
import os
import argparse
import struct
import sys
import copy
import numpy as np
import rospy
import rospkg
import csv

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    PoseArray,
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from tf.transformations import *

APVimage_Filepath = os.path.dirname(os.path.realpath(__file__)) + "/../../../action_primitive_variation/images/"
APVdata_Filepath = os.path.dirname(os.path.realpath(__file__)) + "/../../../action_primitive_variation/data/"
PDDLdata_Filepath = os.path.dirname(os.path.realpath(__file__)) + "/../../../pddl/data/"

def writeToDomainFile(filePath, _name, _reqs, _types, _preds, _actions):

    define = 'define (domain ' + _name + ')\n\n'

    # maybe use join instead 
    reqs = '(:requirements'
    for r in _reqs:
        reqs = reqs + ' ' + r
    reqs = reqs + ')\n\n'

    types = '(:types\n' 
    for t in _types:
        types = types + '    ' + t + '\n'
    types = types + ')\n\n'

    preds = '(:predicates\n'
    for p in _preds:
        preds = preds + '    ' + p + '\n'
    preds = preds + ')\n\n'

    actions = ''
    for a in _actions:
        actions = actions + a + '\n\n'

    # open file 
    with open(filePath, 'w') as f:
        f.write('(')
        f.write(define)
        f.write(reqs)
        f.write(types)
        f.write(preds)
        f.write(actions)
        f.write(')')

def writeToProblemFile(filePath, _task, _domain, _objs, _init, _goals):

    define = 'define (problem  ' + _task + ')\n\n'
    domain = '(:domain ' + _domain + ')\n\n'

    # # maybe use join instead 
    objs = '(:objects'
    for o in _objs:
        objs = objs + '\n    ' + o
    objs = objs + '\n)\n\n'

    init = '(:init' 
    for i in _init:
        init = init + '\n    ' + i
    init = init + '\n)\n\n'

    if len(_goals) > 1:
        goals = '(:goal (and '
        for g in _goals:
            goals = goals + '\n    ' + g
        goals = goals + ')\n)\n\n'
    else:
        goals = '(:goal ' + _goals[0] + ')\n\n'

    # # open file 
    with open(filePath, 'w') as f:
        f.write('(')
        f.write(define)
        f.write(domain)
        f.write(objs)
        f.write(init)
        f.write(goals)
        f.write(')')


def getPlanFromSolutionFile(filePath):
    plan_data = None
    plan = []
    if (os.path.exists(filePath)):
        with open(filePath) as f:
            plan_data = f.readlines()

        for full_action in plan_data:
            data = full_action.replace(")\n", "").replace("(", "")
            args = data.split()
            action = {}
            params = []
            action['actionName'] = args[0]
            for p in range(len(args) -1):
                if args[p+1] is not None: 
                    params.append(args[p+1])
            
            action['params'] = params
            plan.append(action)
    return plan

def saveFigureToImage(fig, filename, loc):
    if loc == 'APV':
        fig.savefig(str(APVimage_Filepath) + str(filename))


def deleteAllPddlFiles():
    for the_file in os.listdir(PDDLdata_Filepath):
        file_path = os.path.join(PDDLdata_Filepath, the_file)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
        except Exception as e:
            print(e)


def deleteAllAPVFiles():
    for the_file in os.listdir(APVdata_Filepath):
        file_path = os.path.join(APVdata_Filepath, the_file)
        # try:
        if os.path.isfile(file_path):
            os.unlink(file_path)
        # except Exception as e:
        #     print(e)
    for the_file in os.listdir(APVimage_Filepath):
        file_path = os.path.join(APVimage_Filepath, the_file)
        # try:
        if os.path.isfile(file_path):
            os.unlink(file_path)
        # except Exception as e:
        #     print(e)


def writeBagData(data, APVtrialName):
    filePath = APVdata_Filepath + APVtrialName
    bagData_fp = filePath + '_bag.csv'
    CPs_fp = filePath + '_cps.csv'
    CPs_filtered_fp = filePath + '_cps_filtered.csv'

    with open(bagData_fp, 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(data['gripper_x'])
        writer.writerow(data['gripper_y'])
        writer.writerow(data['gripper_z'])

    with open(CPs_fp, 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerows(data['groupedCps'])

    with open(CPs_filtered_fp, 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(data['filteredCps'])

def readBagData(APVtrialName):
    filePath = APVdata_Filepath + APVtrialName
    bagData_fp = filePath + '_bag.csv'
    CPs_fp = filePath + '_cps.csv'
    CPs_filtered_fp = filePath + '_cps_filtered.csv'


    bagData = []
    cps = []
    cps_filtered = []

    with open(bagData_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            bagData.append(row)

    with open(CPs_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            cps.append(row)

    with open(CPs_filtered_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            cps.append(row)

    data = {}
    data['bagData'] = bagData
    data['cps'] = cps
    data['cps_filtered'] = cps_filtered
    return data

def processLogData(filePath, logDataList, outputMode='log'):   
    if outputMode == 'log': 
        with open(filePath, 'w') as f:
            for row in logDataList:
                f.write(str(row))
                f.write('\n')
    else:
        for row in logDataList:
            print(str(row))
            print('\n')


def writeEndpointData(data, fieldName):
    filePath = APVdata_Filepath + fieldName
    endptData_fp = filePath + '_endpt.csv'

    with open(endptData_fp, 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(data.x)
        writer.writerow(data.y)
        writer.writerow(data.z)