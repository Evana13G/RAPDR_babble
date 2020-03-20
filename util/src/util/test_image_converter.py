

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

HUE_RANGE = 20
WHITE_THRESH = 15
BLACK_THRESH = 25

def color_count_pixels(frame):
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Black pixels 
    lower_black = np.array([0,0,0], dtype=np.uint8)
    upper_black = np.array([179,255,0 + BLACK_THRESH], dtype=np.uint8)
    black_mask = cv2.inRange(hsv_image, lower_black, upper_black)
    print("Number of black pixels: ")
    print(np.count_nonzero(black_mask))

    # White pixels 
    lower_white = np.array([0,0,255 - WHITE_THRESH], dtype=np.uint8)
    upper_white = np.array([179,0 + WHITE_THRESH,255], dtype=np.uint8)
    white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
    print("Number of white pixels: ")
    print(np.count_nonzero(white_mask))

    # Blue pixels 
    blue = np.uint8([[[255, 0, 0]]]) # insert the bgr values which you want to convert to hsv
    hsvBlue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([hsvBlue[0][0][0] - HUE_RANGE ,100,100], dtype=np.uint8)
    upper_blue = np.array([hsvBlue[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)
    blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
    
    print("Number of blue pixels: ")
    print(np.count_nonzero(blue_mask))

    # Green pixels 
    green = np.uint8([[[0, 255, 0]]]) # insert the bgr values which you want to convert to hsv
    hsvGreen = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
    lower_green = np.array([hsvGreen[0][0][0] - HUE_RANGE,100,100], dtype=np.uint8)
    upper_green = np.array([hsvGreen[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)

    green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
    print("Number of green pixels: ")
    print(np.count_nonzero(green_mask))

    # Red pixels 
    red = np.uint8([[[0, 0, 255]]]) # insert the bgr values which you want to convert to hsv
    hsvRed = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)
    lower_red = np.array([hsvRed[0][0][0] - HUE_RANGE,100,100], dtype=np.uint8)
    upper_red = np.array([hsvRed[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)

    red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
    print("Number of red pixels: ")
    print(np.count_nonzero(red_mask))

    # Orange pixels 
    orange = np.uint8([[[0, 165, 255]]]) # insert the bgr values which you want to convert to hsv
    hsvOrange = cv2.cvtColor(orange, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([hsvOrange[0][0][0] - HUE_RANGE,100,100], dtype=np.uint8)
    upper_orange = np.array([hsvOrange[0][0][0] + HUE_RANGE,255,255], dtype=np.uint8)

    orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
    print("Number of orange pixels: ")
    print(np.count_nonzero(orange_mask))



def process_img(image):
    color_count_pixels(image)
    cv2.imshow('frame', image) # debugging purposes 
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print("###########")

def test():
    print("Testing image converter ...")
    height = 100
    width = 100

    # Black image
    print("Test 1: Black image")
    image = np.zeros((height,width,3), np.uint8)
    process_img(image)

    # White image 
    print("Test 2: White image")
    image[:,:] = (255,255,255)      # (B, G, R)
    process_img(image)

    # Green image
    print("Test 3: Green image")
    image[:,:] = (0,255,0)      # (B, G, R)
    process_img(image)

    # Blue image
    print("Test 4: Blue image")
    image[:,:] = (255,0,0)      # (B, G, R)
    process_img(image)

    # Light Green image 
    print("Test 5: Light green image")
    image[:,:] = (3,252,160)      # (B, G, R)
    process_img(image)

    # Off White image
    print("Test 6: Off white image")
    image[:,:] = (247,247,250)      # (B, G, R)
    process_img(image)

    # Light Black image 
    print("Test 7: Light black image")
    image[:,:] = (22,22,22)      # (B, G, R)
    process_img(image)

test()