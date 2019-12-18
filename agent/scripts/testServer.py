#!/usr/bin/env python


from agent.srv import *
from std_msgs.msg import (
    Empty,
)
import rospy
from environment.srv import HandleEnvironmentSrv
import time



def main():
    rospy.init_node("agent_test_node")

    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.wait_for_service('brain_A_srv', timeout=60)
    # rospy.wait_for_service('brain_B_srv', timeout=60)
    rospy.wait_for_service('init_environment', timeout=60)

    env = rospy.ServiceProxy('init_environment', HandleEnvironmentSrv)

    reg_trials = 4

    try:
        brain_A = rospy.ServiceProxy('brain_A_srv', BrainSrv)
        # brain_B = rospy.ServiceProxy('brain_B_srv', BrainSrv)

        print("********************************")
        print("Running REG TRIAL A EXPERIMENTS\n")
        # for i in range(reg_trials):
        # print("Trial # " + str(i) + ', Brain A')
        # testName = 'armData_' + str(i)
        testName = 'armData_1' 
        resp_A = brain_A(testName, 100, 10)
        print(resp_A.timePerAttempt)
        print(resp_A.totalTime)
        # env('restart')
        time.sleep(5)

        # print("********************************")
        # print("Running REG TRIAL B EXPERIMENTS\n")
        # for i in range(reg_trials):
        #     print("Trial # " + str(i) + ', Brain B')
        #     testName = 'TEST_' + str(i)
        #     resp_B = brain_B(testName, 100, 10)
        #     print(resp_B.timePerAttempt)
        #     print(resp_B.totalTime)
        #     env('restart')
        

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
