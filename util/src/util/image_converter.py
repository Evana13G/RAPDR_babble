

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

# List of objects and their color 
  # Cup, white 
  # Cover, green 

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.initTime = 0
        self.savedFrames = {}
        self.savedFramesStr = ""
        self.image_sub = rospy.Subscriber("/cameras/head_camera/image", Image, self.callbackImage)
        self.block_pixels = 0
        self.cup_pixels = 0

    def callbackImage(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        frame = cv_image

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        ######### Pixels for cover #################
        # Threshold the HSV image to get only green colors 
        green = np.uint8([[[0, 255, 0]]]) # insert the bgr values which you want to convert to hsv
        hsvGreen = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
        #print(hsvGreen)

        lower_green = hsvGreen[0][0][0] - 10, 100, 100
        upper_green = hsvGreen[0][0][0] + 10, 255, 255

        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        self.cover_pixels = np.count_nonzero(green_mask)
        # previous values ...
        # lower_green = np.array([40,40,40])
        # upper_green = np.array([200,255,255])
        
        ######### Pixels for cup #################
        # Threshold the HSV image to get only white colors 
        white = np.uint8([[[255, 255, 255]]]) # insert the bgr values which you want to convert to hsv
        hsvWhite = cv2.cvtColor(white, cv2.COLOR_BGR2HSV)
        lower_white = hsvWhite[0][0][0] - 10, 100, 100
        upper_whiteS = hsvWhite[0][0][0] + 10, 255, 255
        white_mask = cv2.inRange(hsv, lower_green, upper_green)
        self.cup_pixels = np.count_nonzero(white_mask)


        # lower_red = np.array([0,0,255])
        # upper_red = np.array([150,150, 255])

    def getObjectPixelCount(self, object):
      if (object == 'cover'):
        return self.cover_pixels
      elif (object == 'cup'):
        return self.cup_pixels
      else:
        return 0