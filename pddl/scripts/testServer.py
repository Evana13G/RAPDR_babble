#!/usr/bin/env python

from pddl.srv import *
import rospy
import os 

# from std_msgs.msg import (
#     Header,
#     Empty,
# )

# from action_primitive_variation.srv import APVSrv

def main():
    rospy.init_node("pddl_test_node")
    rospy.wait_for_service('plan_executor_srv', timeout=60)
    try:
        filepath = os.path.dirname(os.path.realpath(__file__)) + "/problem_1.pddl.soln"
        b = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)
        # resp = b('obtain_object', 'left', 'block', None)
        resp = b(filepath)
        print("Response: ")
        print(resp)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
