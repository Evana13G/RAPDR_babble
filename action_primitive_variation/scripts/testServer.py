#!/usr/bin/env python

from agent.srv import PressButtonSrv, CloseGripperSrv, OpenGripperSrv, ObtainObjectSrv
import rospy

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

from action_primitive_variation.srv import APVSrv
from util.bayesian_change_point import BayesianChangePoint
import numpy as np

# import cpdetect 
# import sys, argparse, csv
import matplotlib.pyplot as plt # if no vis, delete  
import pandas as pd
from sklearn.cluster import AgglomerativeClustering

import scipy.cluster.hierarchy as shc


def main():
    rospy.init_node("APV_test_node")
    rospy.wait_for_service('APV_srv', timeout=60)
    try:
        b = rospy.ServiceProxy('APV_srv', APVSrv)
        # resp = b('obtain_object', 'left', 'block', None)
        resp = b('press_button', 'left_gripper', 'left_button', None)
        print(resp)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)




if __name__ == "__main__":
    main()
