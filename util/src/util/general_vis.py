#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import matplotlib.cbook as cbook

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


def generateEndptImage(visData, visTitle):
    fig = plt.figure()
    x = [elem.x for elem in visData]
    y = [elem.y for elem in visData]
    z = [elem.z for elem in visData]

    plt.plot(x, 'r--', label='x')
    plt.plot(y, 'g', label='y')
    plt.plot(z, 'b--', label='z')
    plt.title(visTitle)
    # saveFigureToImage(fig, imageFilePath, 'APV')
    # plt.clf(fig)
    # plt.close(fig)

    plt.show()

def generateJointImage(visData, visLabels, visTitle, indices):

    colrs = ['m', 'c', 'y', '#a3c2c2', '#ff9933', '#aa80ff', 'r', 'b', 'chartreuse']

    fig = plt.figure()
    for i in range(len(visData)):
        clr = colrs[i % (len(colrs)-1)]
        plt.plot(visData[i], clr, label=visLabels[i])
    plt.legend()
    plt.title(visTitle)


    for new_artion in indices:
        plt.axvline(x=new_artion, color='k')

    plt.show()




def generate_RAPDR_babble_viz(filename):

    
    total_times = np.array([1582.93,2963.549,2013.145,2825.344,1658.51,1862.413,3406.727,1779.954,1513.791,1650.813,1828.624])
    num_trails = np.array([11,20,14,18,11,13,20,12,10,11,13])
    avg_trial_times = np.array([143.903,148.178,143.8,156.964,150.774,143.263,170.336,148.33,151.379,150.074,140.663])
    success_actions = ['shake-movementmagnitude:5.0','shake-rate:10.5','shake-rate:10.5','shake-rate:20.0','shake-orientation:top','shake-orientation:top','push-orientation:top','shake-rate:20.0','shake-movementmagnitude:5.0','shake-rate:20.0','shake-orientation:top']
    std_devs = [77.599,65.003,67.997,60.609,55.674,54.333,76.043,82.258,70.495,57.296,53.961]

    colors = ['r','g','g','b','y','y','w','b','r','b','w']
    color_labels = [('r', 'shake-movementmagnitude:5.0'), 
                    ('g', 'shake-rate:10.5'),
                    ('b', 'shake-rate:20.0'),
                    ('y', 'shake-orientation:top'),
                    ('w', 'push-orientation:top')]

    fig, ax = plt.subplots()
    ax.scatter(num_trails,avg_trial_times, c=colors, s=std_devs, alpha=0.5)

    ax.set_xlabel(r'$\Delta_i$', fontsize=15)
    ax.set_ylabel(r'$\Delta_{i+1}$', fontsize=15)
    ax.set_title('Volume and percent change')

    ax.grid(True)
    fig.tight_layout()

    plt.show()


generate_RAPDR_babble_viz()


