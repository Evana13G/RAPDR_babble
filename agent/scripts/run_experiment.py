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
    demo_mode = req.demo_mode
    results = {'discover_strike' : {'exploration_times' : [], 'total_time': []},
               'cook' : {'exploration_times' : [], 'total_time': []}}

    if demo_mode == True:
        num_discover_strike_runs = 1
        num_cook_runs = 1

    try:
        
        # Discover Strike
        for i in range(num_discover_strike_runs):
            run_name = experimentName + '_' + str(i)
            result = BrainProxy(run_name, 'discover_strike', demo_mode)
            rospy.sleep(1)
            exploration_times = result.timePerAttempt
            total_time = result.totalTime
            results['discover_strike']['exploration_times'].append(exploration_times)
            results['discover_strike']['total_time'].append(total_time)
        rospy.sleep(1)

        # Cook
        for i in range(num_cook_runs):
            run_name = experimentName + '_' + str(i)
            result = BrainProxy(run_name, 'cook', demo_mode)
            rospy.sleep(1)
            exploration_times = result.timePerAttempt
            total_time = result.totalTime
            results['cook']['exploration_times'].append(exploration_times)
            results['cook']['total_time'].append(total_time)

        rospy.sleep(1)

        experiment_complete = True
        print(results)

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        experiment_complete = False

    return RunExperimentSrvResponse(experiment_complete)

def main():
    rospy.init_node("experiments_node")
    # rospy.wait_for_service('/brain_srv', timeout=60)
    rospy.Service("experiments_srv", RunExperimentSrv, run_experiments)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()