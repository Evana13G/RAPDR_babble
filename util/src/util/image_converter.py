

from __future__ import print_function, division
import roslib
#roslib.load_manifest('my_package')
import sys
import cv2
import math
import time
from cv_bridge import CvBridge, CvBridgeError

import argparse
import struct
import sys
import copy
import numpy as np
import rospy
import rospkg
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    GetLinkState,
)
from gazebo_msgs.msg import (
    LinkState,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    PoseArray,
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
    
)
from sensor_msgs.msg import (
    Image,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from tf.transformations import *
import baxter_interface
from environment.srv import *


class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.initTime = 0
        self.savedFrames = {}
        self.savedFramesStr = ""
        self.image_sub = rospy.Subscriber("/cameras/head_camera/image", Image, self.callbackImage)
        self.block_pixels = 0

    def callbackImage(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        frame = cv_image
        #cv2.imshow('frame', frame)
#        frame = cv2.resize(frame,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_CUBIC)
#        frameCopy = frame.copy()
#        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40,40,40])
        upper_green = np.array([200,255,255])
#        lower_red = np.array([0,0,255])
#        upper_red = np.array([150,150, 255])
#
#        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        self.block_pixels = np.count_nonzero(mask)



       # contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
       # for cnt in contours:
       #     approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
       #     #print(len(approx))
       #     if len(approx)==4:
       #         print("square")
       #         cv2.drawContours(frameCopy,[cnt],0,(0,0,255),-1)
       # maskRect = cv2.inRange(frameCopy, lower_red, upper_red)

       # kernel = np.ones((5,5), np.uint8)
       # erosion = cv2.erode(maskRect, kernel, iterations = 1)
       # erosionArray = np.asarray(erosion)
       # areaErosion = np.count_nonzero(erosionArray)

    def getBlockPixelCount(self):
        return self.block_pixels