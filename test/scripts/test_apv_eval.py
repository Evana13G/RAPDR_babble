#!/usr/bin/env python

import rospy
import time

from agent.srv import *
from environment.srv import *
from pddl.srv import *
from action_primitive_variation.srv import *

from util.goal_management import *

#### Service Proxies

APVproxy = rospy.ServiceProxy('APV_srv', APVSrv)

def test(req):
    print("--------------------------------------")
    print("----- TESTING APV EVAL -----")

    try:
        
        resp = APVproxy('push', ['left_gripper', 'cover'], 'orientation', 5, 'default')

        return True 
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return False 

def main():
    rospy.init_node("test_apv_eval")
    rospy.Service("test_apv_eval_srv", EmptyTestSrv, test)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()


