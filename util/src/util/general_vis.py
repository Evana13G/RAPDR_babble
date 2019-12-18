#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from util.bayesian_change_point import BayesianChangePoint
from util.ros_bag import RosBag
from util.file_io import * 

import csv


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

    # saveFigureToImage(fig, imageFilePath, 'APV')
    # plt.clf(fig)
    # plt.close(fig)

    plt.show()

def generateVisData_bagAndCPData(bagName, bagData, cpData):
    visualizableData = {}
    if (bagName == "leftGripper") or (bagName == "rightGripper"):
        visualizableData['gripper_x'] = bagData[0]
        visualizableData['gripper_y'] = bagData[1]
        visualizableData['gripper_z'] = bagData[2]
        visualizableData['groupedCps'] = cpData.getGroupedData()
        visualizableData['filteredCps'] = cpData.getCompressedChangePoints()
    else:
        print("No data passed")
    return visualizableData


