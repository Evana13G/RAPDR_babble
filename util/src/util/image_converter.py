#!/usr/bin/env python

import rospy

import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import (
    Image,
    PointCloud2
)

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
        # self.kinetic_sub = rospy.Subscriber("/kinect_camera/rgb/image_raw", Image, self.callbackKineticImage)

        # Color pixel count 
        self.black_pixels = 0
        self.white_pixels = 0
        self.blue_pixels = 0
        self.green_pixels = 0
        self.red_pixels = 0
        self.orange_pixels = 0

        self.color_ranges = {}

    def callbackImage(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame = cv_image
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        self.color_count_pixels(hsv_image)

    # def callbackKineticImage(self, data):
    #     # print(type(data))
    #     # flags = [i for i in dir(cv2) if i.startswith('COLOR_')] # Origin to target space
    #     # print(len(flags))
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
    #     except CvBridgeError as e:
    #         print(e)
    #     if cv_image is not None:
    #         frame = cv_image
    #         hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    def color_count_pixels(self, hsv_image):
        # Black pixels 
        lower_black = np.array([0,0,0], dtype=np.uint8)
        upper_black = np.array([179,255,0 + BLACK_THRESH], dtype=np.uint8)
        black_mask = cv2.inRange(hsv_image, lower_black, upper_black)
        self.black_pixels = np.count_nonzero(black_mask)
        self.black_segments = self.count_segmented_areas(black_mask)

        # White pixels 
        lower_white = np.array([0,0,255 - WHITE_THRESH], dtype=np.uint8)
        upper_white = np.array([179,0 + WHITE_THRESH,255], dtype=np.uint8)
        white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
        self.white_pixels = np.count_nonzero(white_mask)
        self.white_segments = self.count_segmented_areas(white_mask)

        # Blue pixels 
        blue = np.uint8([[[255, 0, 0]]]) # insert the bgr values which you want to convert to hsv
        hsvBlue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([hsvBlue[0][0][0] - HUE_RANGE ,100,100], dtype=np.uint8)
        upper_blue = np.array([hsvBlue[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        self.blue_pixels = np.count_nonzero(blue_mask)
        self.blue_segments = self.count_segmented_areas(blue_mask)

        # Green pixels 
        green = np.uint8([[[0, 255, 0]]]) # insert the bgr values which you want to convert to hsv
        hsvGreen = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
        lower_green = np.array([hsvGreen[0][0][0] - HUE_RANGE,100,100], dtype=np.uint8)
        upper_green = np.array([hsvGreen[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        self.green_pixels = np.count_nonzero(green_mask)
        # self.green_segments = self.count_segmented_areas(green_mask)

        # Red pixels 
        red = np.uint8([[[0, 0, 255]]]) # insert the bgr values which you want to convert to hsv
        hsvRed = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)
        lower_red = np.array([hsvRed[0][0][0] - HUE_RANGE,100,100], dtype=np.uint8)
        upper_red = np.array([hsvRed[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)
        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
        self.red_pixels = np.count_nonzero(red_mask)
        self.red_segments = self.count_segmented_areas(red_mask)

        # Orange pixels 
        orange = np.uint8([[[0, 165, 255]]]) # insert the bgr values which you want to convert to hsv
        hsvOrange = cv2.cvtColor(orange, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([hsvOrange[0][0][0] - HUE_RANGE,100,100], dtype=np.uint8)
        upper_orange = np.array([hsvOrange[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)
        orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
        self.orange_pixels = np.count_nonzero(orange_mask)
        self.orange_segments = self.count_segmented_areas(orange_mask)


    def count_segmented_areas(self, mask):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        counts = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        counts = counts[0] if len(counts) == 2 else counts[1]
        return len(counts)

    def is_visible(self, obj):
        if (obj == 'cover'):
            return self.green_pixels > 0
        elif (obj == 'cup'):
            return self.blue_pixels > 0
        elif (obj == 'burner1'):
            return self.red_pixels > 0
        # elif (obj == 'burner1'):
        #     return self.red_pixels > 0
        else:
            return False

    def getObjectPixelCount(self, obj):
        if (obj == 'cover'):
            return self.green_pixels
        elif (obj == 'cup'):
            return self.blue_pixels 
        else:
            return 0