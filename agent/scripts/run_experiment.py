#!/usr/bin/env python

import rospy
from agent.srv import *
from util.goal_management import *

BrainProxy = rospy.ServiceProxy('brain_srv', BrainSrv)

def run_experiments(req):
    print("\n########################################################")
    print("########################################################")
    print('##                                                    ##')
    print('##   Action Primitive Discovery in Robotic Agents     ##') 
    print('##         through Action Parameter Variation         ##')
    print('##                                                    ##')
    print('## -- a proof of concept model for knowledge          ##')
    print('##    acquisition in intelligent agents, 2021         ##')
    print('##                                                    ##')
    print("########################################################")
    print("########################################################\n")

    experimentName = req.experiment_name
    num_discover_strike_runs = req.num_discover_strike_runs
    num_cook_runs = req.num_cook_runs

    try:
        
        # Discover Strike
        for i in range(num_discover_strike_runs):
            run_name = experimentName + '_' + str(i)
            results = BrainProxy(run_name, 'discover_strike')
            rospy.sleep(1)
            exploration_times = req.timePerAttempt
            total_time = req.totalTime
        rospy.sleep(1)

        # Discover Strike
        for i in range(num_discover_strike_runs):
            run_name = experimentName + '_' + str(i)
            results = BrainProxy(run_name, 'cook')
            rospy.sleep(1)
            exploration_times = req.timePerAttempt
            total_time = req.totalTime
        rospy.sleep(1)

        experiment_complete = True

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        experiment_complete = False

    return RunExperimentSrvResponse(experiment_complete)

def main():
    rospy.init_node("experiments_node")
    rospy.Service("experiments_srv", RunExperimentSrv, run_experiments)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()