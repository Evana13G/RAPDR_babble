#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg
import rosbag

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from util.data_conversion import * 
import matplotlib.pyplot as plt
class RosBag(object):
    def __init__(self, bagName):
        self.name = bagName
        self.bag = rosbag.Bag(bagName+'.bag','w')

    # def getNumTrajs(self):
    #     if self.name == 'jointState':
    #         return 7
    #     else:
    #         return 0

    def getBag(self):
        return self.bag

    def closeBag(self):
        self.bag.close()

    def openBag(self):
        self.bag = rosbag.Bag(self.name+'.bag','w')

    def writeToBag(self, topic, data):
        self.bag.write(topic, data)

    # def getBagDataAtCps(self, changePoints, _topics):
    #     # ['robot/joint_states']
    #     openBag()
        
    #     data = []
    #     for topic, msg, t in self.bag.read_messages(topics=_topics):
    #         data.append(msg)
    #     return data

    def getROSBagDataAtCps(self, bag, topics, cps):
        allMsgs = []
        allTimes = []
        for topic, msg, t in self.bag.read_messages(topics=topics):
            allMsgs.append(msg)
            allTimes.append(t)

        gripperInfo = []
        for pt in cps:
            gripperInfo.append(allMsgs[int(pt)])

        return gripperInfo

    def getVisualizableData(self):
        # Indexes
        if self.name == 'jointState':

            # hardcoded indices 
            i_left_e0 = 3
            i_left_e1 = 4
            i_left_s0 = 5
            i_left_s1 = 6
            i_left_w0 = 7
            i_left_w1 = 8
            i_left_w2 = 9
            i_right_e0 = 12
            i_right_e1 = 13
            i_right_s0 = 14
            i_right_s1 = 15
            i_right_w0 = 16
            i_right_w1 = 17
            i_right_w2 = 18

            # Arrays of vals to return 
            left_e0 = []
            left_e1 = []
            left_s0 = []
            left_s1 = []
            left_w0 = []
            left_w1 = []
            left_w2 = []
            right_e0 = []
            right_e1 = []
            right_s0 = []
            right_s1 = []
            right_w0 = []
            right_w1 = []
            right_w2 = []


            # for topic, msg, t in self.bag.read_messages(topics=['robot/joint_states', 'left_gripper_pose']):
            for topic, msg, t in self.bag.read_messages():
                if topic == 'robot/joint_states':
                    left_e0.append(msg.position[i_left_e0])
                    left_e1.append(msg.position[i_left_e1])
                    left_s0.append(msg.position[i_left_s0])
                    left_s1.append(msg.position[i_left_s1])
                    left_w0.append(msg.position[i_left_w0])
                    left_w1.append(msg.position[i_left_w1])
                    left_w2.append(msg.position[i_left_w2])
                    right_e0.append(msg.position[i_right_e0])
                    right_e1.append(msg.position[i_right_e1])
                    right_s0.append(msg.position[i_right_s0])
                    right_s1.append(msg.position[i_right_s1])
                    right_w0.append(msg.position[i_right_w0])
                    right_w1.append(msg.position[i_right_w1])
                    right_w2.append(msg.position[i_right_w2])
                
            halfLen = len(left_e0)/2

            left_e0 = left_e0[:halfLen]
            left_e1 = left_e1[:halfLen]
            left_s0 = left_s0[:halfLen]
            left_s1 = left_s1[:halfLen]
            left_w0 = left_w0[:halfLen]
            left_w1 = left_w1[:halfLen]
            left_w2 = left_w2[:halfLen]
            right_e0 = right_e0[:halfLen]
            right_e1 = right_e1[:halfLen]
            right_s0 = right_s0[:halfLen]
            right_s1 = right_s1[:halfLen]
            right_w0 = right_w0[:halfLen]
            right_w1 = right_w1[:halfLen]
            right_w2 = right_w2[:halfLen]

            # print(type(left_e0[0]))
            return [left_e0, left_e1, left_s0, left_s1, left_w0, left_w1, left_w2]
                # right_e0, right_e1, right_s0, right_s1, right_w0, right_w1, right_w2]

        elif self.name == 'predicate':
            
            # obj_loc_x = []
            # obj_loc_y = []
            # obj_loc_z = []
            l_btn_pressed = []
            r_btn_pressed = []
            obj_vis = []

 
            for topic, msg, t in self.bag.read_messages(topics=['predicate_values']):
                stringFormatList = pddlStringFormat(msg.predicates)
                
                # for pred in msg.predicates:
                #     if pred.operator == "at":
                #         if pred.object == "left_gripper":

                            # if((round(pred.locationInformation.pose.position.x, 2) != 0.00) and 
                            #    (round(pred.locationInformation.pose.position.y, 2) != 0.00) and 
                            #    (round(pred.locationInformation.pose.position.z, 2) != 0.00)):

                            # obj_loc_x.append(round(pred.locationInformation.pose.position.x, 2))
                            # obj_loc_y.append(round(pred.locationInformation.pose.position.y, 2))
                            # obj_loc_z.append(round(pred.locationInformation.pose.position.z, 2))

                if "pressed(left_button)" in stringFormatList:
                    l_btn_pressed.append(9)
                else: 
                    l_btn_pressed.append(8.5)
                if "pressed(right_button)" in stringFormatList:
                    r_btn_pressed.append(9)
                else: 
                    r_btn_pressed.append(8.5)
                if "is_visible(block)" in stringFormatList:
                    obj_vis.append(9)
                else: 
                    obj_vis.append(8.5)

            l_btn_pressed = l_btn_pressed[0::100]
            r_btn_pressed = r_btn_pressed[0::100]
            obj_vis = obj_vis[0::100]
            
            plt.plot(l_btn_pressed, 'r', label='l_btn_pressed')
            plt.plot(r_btn_pressed, 'b', label='r_btn_pressed')
            plt.plot(obj_vis, 'g', label='obj_vis')

            plt.show()
            
            # halfLen = len(left_e0)/2

            # left_e0 = left_e0[:halfLen]
            # left_e1 = left_e1[:halfLen]
            # left_s0 = left_s0[:halfLen]
            # left_s1 = left_s1[:halfLen]
            # left_w0 = left_w0[:halfLen]
            # left_w1 = left_w1[:halfLen]
            # left_w2 = left_w2[:halfLen]
            # right_e0 = right_e0[:halfLen]
            # right_e1 = right_e1[:halfLen]
            # right_s0 = right_s0[:halfLen]
            # right_s1 = right_s1[:halfLen]
            # right_w0 = right_w0[:halfLen]
            # right_w1 = right_w1[:halfLen]
            # right_w2 = right_w2[:halfLen]

            # print(type(obj_loc_x[0]))
            # return [obj_loc_x, obj_loc_y, obj_loc_z, 
            return [l_btn_pressed]
            # , r_btn_pressed, obj_vis]
        elif self.name == "leftGripper":

            lg_x = []
            lg_y = []
            lg_z = []

 
            for topic, msg, t in self.bag.read_messages(topics=['left_gripper_pose']):

                # print("Time?:  " + str(t))

                lg_x.append(msg.pose.position.x)
                lg_y.append(msg.pose.position.y)
                lg_z.append(msg.pose.position.z)


            halfLen = len(lg_x)/2

            lg_x = lg_x[:halfLen]
            lg_y = lg_y[:halfLen]
            lg_z = lg_z[:halfLen]

            # plt.plot(lg_x, 'r', label='x')
            # plt.plot(lg_y, 'b', label='y')
            # plt.plot(lg_z, 'g', label='z')
            # plt.show()

            return [lg_x, lg_y, lg_z]

        elif self.name == "rightGripper":

            rg_x = []
            rg_y = []
            rg_z = []

 
            for topic, msg, t in self.bag.read_messages(topics=['right_gripper_pose']):

                # print("Time?:  " + str(t))

                rg_x.append(msg.pose.position.x)
                rg_y.append(msg.pose.position.y)
                rg_z.append(msg.pose.position.z)


            halfLen = len(rg_x)/2

            rg_x = rg_x[:halfLen]
            rg_y = rg_y[:halfLen]
            rg_z = rg_z[:halfLen]

            return [rg_x, rg_y, rg_z]
        else:
            return []


