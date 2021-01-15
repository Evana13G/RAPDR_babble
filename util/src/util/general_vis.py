#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

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




def generate_RAPDR_babble_viz():
    # Load a numpy record array from yahoo csv data with fields date, open, close,
    # volume, adj_close from the mpl-data/example directory. The record array
    # stores the date as an np.datetime64 with a day unit ('D') in the date column.
    trial_data = [77.59876737,5.00294507,67.99704903,60.60885018,55.67428004,54.33326712,76.04294074,82.25771159,70.4953777,57.29577446, 53.96076964]

    delta1 = np.diff(trial_data.adj_close) / trial_data.adj_close[:-1]

    # Marker size in units of points^2
    volume = (15 * trial_data.volume[:-2] / trial_data.volume[0])**2
    close = 0.003 * trial_data.close[:-2] / 0.003 * trial_data.open[:-2]

    fig, ax = plt.subplots()
    ax.scatter(delta1[:-1], delta1[1:], c=close, s=volume, alpha=0.5)

    ax.set_xlabel(r'$\Delta_i$', fontsize=15)
    ax.set_ylabel(r'$\Delta_{i+1}$', fontsize=15)
    ax.set_title('Volume and percent change')

    ax.grid(True)
    fig.tight_layout()

    plt.show()