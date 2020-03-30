

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

HUE_RANGE = 20
WHITE_THRESH = 15
BLACK_THRESH = 25

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.initTime = 0
        self.savedFrames = {}
        self.savedFramesStr = ""
        self.image_sub = rospy.Subscriber("/cameras/head_camera/image", Image, self.callbackImage)
        # Color pixel count 
        self.black_pixels = 0
        self.white_pixels = 0
        self.blue_pixels = 0
        self.green_pixels = 0
        self.red_pixels = 0
        self.orange_pixels = 0

    def callbackImage(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        frame = cv_image
        cv2.imshow('frame', frame) # debugging purposes 
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color_count_pixels(hsv_image)


    def color_count_pixels(hsv_image):
      # Black pixels 
      lower_black = np.array([0,0,0], dtype=np.uint8)
      upper_black = np.array([179,255,0 + BLACK_THRESH], dtype=np.uint8)
      black_mask = cv2.inRange(hsv_image, lower_black, upper_black)
      self.black_pixels = np.count_nonzero(black_mask)

      # White pixels 
      lower_white = np.array([0,0,255 - WHITE_THRESH], dtype=np.uint8)
      upper_white = np.array([179,0 + WHITE_THRESH,255], dtype=np.uint8)
      white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
      self.white_pixels = np.count_nonzero(white_mask)

      # Blue pixels 
      blue = np.uint8([[[255, 0, 0]]]) # insert the bgr values which you want to convert to hsv
      hsvBlue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
      lower_blue = np.array([hsvBlue[0][0][0] - HUE_RANGE ,100,100], dtype=np.uint8)
      upper_blue = np.array([hsvBlue[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)
      blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
      self.blue_pixels = np.count_nonzero(blue_mask)

      # Green pixels 
      green = np.uint8([[[0, 255, 0]]]) # insert the bgr values which you want to convert to hsv
      hsvGreen = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
      lower_green = np.array([hsvGreen[0][0][0] - HUE_RANGE,100,100], dtype=np.uint8)
      upper_green = np.array([hsvGreen[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)
      green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
      self.green_pixels = np.count_nonzero(green_mask)

      # Red pixels 
      red = np.uint8([[[0, 0, 255]]]) # insert the bgr values which you want to convert to hsv
      hsvRed = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)
      lower_red = np.array([hsvRed[0][0][0] - HUE_RANGE,100,100], dtype=np.uint8)
      upper_red = np.array([hsvRed[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)
      red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
      self.red_pixels = np.count_nonzero(red_mask)

      # Orange pixels 
      orange = np.uint8([[[0, 165, 255]]]) # insert the bgr values which you want to convert to hsv
      hsvOrange = cv2.cvtColor(orange, cv2.COLOR_BGR2HSV)
      lower_orange = np.array([hsvOrange[0][0][0] - HUE_RANGE,100,100], dtype=np.uint8)
      upper_orange = np.array([hsvOrange[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)
      orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
      self.orange_pixels = np.count_nonzero(orange_mask)

    def getObjectPixelCount(self, object):
      if (object == 'cover'):
        return self.green_pixels
      elif (object == 'cup'):
        return self.white_pixels
      else:
        return 0