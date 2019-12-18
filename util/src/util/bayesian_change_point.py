#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import rospy
import rospkg

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from action_primitive_variation.srv import *
from agent.srv import *
# from sklearn.svm import LinearSVC
# from sklearn.datasets import make_classification
import cpdetect 
import numpy as np
import sys, argparse, csv
import matplotlib.pyplot as plt # if no vis, delete  
import pandas as pd
from sklearn.cluster import AgglomerativeClustering
import scipy.cluster.hierarchy as shc

class BayesianChangePoint(object):
    def __init__(self, rawData, filename, _clusterThreshold, _clusterMinSize):
        self.clusterThreshold = _clusterThreshold
        self.minGroupSize = _clusterMinSize
        traj_names, cp_set, aggr_cp_list = self.detectChangePoints(rawData, filename)
        self.trajectoryNames = traj_names
        self.trajectoryChangePoints = cp_set
        self.aggregateChangePoints = aggr_cp_list
        clstrs, _groups, _minCps = self.clusterChangePoints(aggr_cp_list)
        self.clusterLabeledMask = clstrs
        self.groupedByClusterLabel = _groups
        self.compressedChangePoints = _minCps



    def detectChangePoints(self, rawData, filename):
        detector = cpdetect.cpDetector(rawData, distribution='normal', log_odds_threshold=1)
        detector.detect_cp()
        detector.to_csv(filename)
        return self.csvExtractCps(filename)

    def csvExtractCps(self, filename):
        content = {}
        content_list = []
        trajs = []
        aggregateCps = []

        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            next(reader)
            for row in reader:
                # get keys
                trajs.append(row[0])
                content_list.append((row[0], float(row[2]))) # tuple 
                aggregateCps.append(row[2])
            trajs = list(set(trajs))
            for traj in trajs:
                content[traj] = []
            for tup in content_list:
                content[tup[0]].append(tup[1])
            return trajs, content, aggregateCps



    def clusterChangePoints(self, points):
        cps = np.array(self.get2DTraj(points))
        clusters = shc.fclusterdata(cps, self.getThreshold(), criterion="distance")
        labels = list(set(clusters))
        
        groups = []
        for label in range(len(labels)):
            groups.append([])
        
        for i in range(len(cps)-1):
            groups[clusters[i]-1].append(float(cps[i][0]))
        
        flattened_cps = []
        for chunk in groups:
            if(len(chunk) > self.minGroupSize):
        #    if(len(chunk) > 3):
                flattened_cps.append(min(chunk))
                flattened_cps.append(max(chunk))
                flattened_cps.append(np.mean(np.array(chunk)))

        return clusters, groups, flattened_cps

    def get2DTraj(self, traj):
        twoDee = []
        for elem in traj:
            twoDee.append([elem, elem])
        return twoDee

    # def getROSBagDataAtCps(self, bag, topics):
    #     print("********************************")
    #     for topic, msg, t in self.bag.read_messages(topics=topics):
    #         print(msg)

##############################################################################################
##################################### SETTERS AND GETTERS ####################################

    def getTrajectoryBasedPoints(self, trajectory):
        return self.trajectoryChangePoints[trajectory]

    def getAggregateChangePoints(self):
        return self.aggregateChangePoints
    
    def getThreshold(self):
        return self.clusterThreshold

    def getClusterLabeledMask(self):
        return self.clusterLabeledMask

    def getGroupedData(self):
        return self.groupedByClusterLabel

    def getCompressedChangePoints(self):
        return self.compressedChangePoints

    def getTrajNames(self):
        return self.trajectoryNames

    def getNumTrajs(self):
        return len(self.numberOfTrajectories)