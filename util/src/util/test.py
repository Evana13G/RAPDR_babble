
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

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import logging

APVimage_Filepath = os.path.dirname(os.path.realpath(__file__)) + "/../../../action_primitive_variation/images/"
APVdata_Filepath = os.path.dirname(os.path.realpath(__file__)) + "/../../../action_primitive_variation/data/"
PDDLdata_Filepath = os.path.dirname(os.path.realpath(__file__)) + "/../../../pddl/data/"
results_Filepath = os.path.dirname(os.path.realpath(__file__)) + "/../../../results/"

def generateImage(visData):

    bagData = visData['bagData']
    cps = visData['cps']
    cps_filtered = visData['cps_filtered']

    fig = plt.figure()

    plt.plot(bagData[0], 'r--', label='x')
    plt.plot(bagData[1], 'g', label='y')
    plt.plot(bagData[2], 'b--', label='z')

    colrs = ['m', 'c', 'y', '#a3c2c2', '#ff9933', '#aa80ff']

    for group_number in range(len(cps) - 1):
        clr = colrs[group_number % (len(colrs)-1)]
        for element in cps[group_number]:
            plt.axvline(x=element, color=clr)

    for stringentCp in cps_filtered:
        plt.axvline(x=stringentCp, color='k')

    plt.show()

def readBagData(APVtrialName):
    filePath = results_Filepath + APVtrialName
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
            cps_filtered = row

    data = {}
    data['bagData'] = bagData
    data['cps'] = cps
    data['cps_filtered'] = cps_filtered
    return data

generateImage(readBagData('test_0/press_buttonright_gripperleft_button_1'))