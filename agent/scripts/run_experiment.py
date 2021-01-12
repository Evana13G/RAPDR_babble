#!/usr/bin/env python

import rospy
from agent.srv import *
from util.goal_management import *

BrainProxy = rospy.ServiceProxy('brain_srv', BrainSrv)
experiments_csv_header = ['scenario', 'run_name', 'total_time', 'num_trails', 'avg_trial_time', 'success_actions']

individual_run_csv_header = ['exploration_times', 'total_time', 'success_plan']
demo_mode = False


#######################################################################
##### HELPER FUNCTIONS          #######################################
def quantify_run_result(scenarioName, run_name, result):
    attempt_times = result.timePerAttempt
    total_time = result.totalTime
    total_attempts = len(attempt_times)
    if len(attempt_times ) > 0:
        average_attempt_time = sum(attempt_times) / len(attempt_times)
    else:
        average_attempt_time = 0.0
    raw_plan = result.success_plan
    novel_names = [a.actionName for a in raw_plan]
    novel_names = [n for n in novel_names if ':' in n]
    formatted_novel_names = str(novel_names).replace(' ', '')

    for_csv = []
    for_csv.append(scenarioName)
    for_csv.append(run_name)
    for_csv.append(total_time)
    for_csv.append(total_attempts)
    for_csv.append(average_attempt_time)
    for_csv.append(formatted_novel_names)

    return for_csv


def format_run_result(result):
    for_csv = []
    for_csv.append(list(result.timePerAttempt))
    for_csv.append(result.totalTime)
    raw_plan = result.success_plan
    formatted_plan = []
    for action in raw_plan:
        formatted_action = []
        formatted_action.append(action.actionName)
        formatted_action.append(action.argVals)
        formatted_plan.append(formatted_action)
    formatted_plan = str(formatted_plan).replace(' ', '')
    for_csv.append(formatted_plan)
    return for_csv
#######################################################################


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
    num_cook_defocused_runs = req.num_cook_defocused_runs

    if req.demo_mode == True:
        num_discover_strike_runs = 1
        num_cook_runs = 1
        num_cook_defocused_runs = 1
        global demo_mode
        demo_mode = True

    try:
        curr_path = os.path.dirname(os.path.realpath(__file__)) 
        experiment_path = generateExperimentDir(curr_path, experimentName)
        experiment_aggregate_file = initResultCsvFile(experiment_path, 'aggregate_results', experiments_csv_header)

        discover_strike_results = run_experiment(experimentName, experiment_path, "discover_strike", num_discover_strike_runs)
        cook_results = run_experiment(experimentName, experiment_path, "cook", num_cook_runs)
        cook_defocused_results = run_experiment(experimentName, experiment_path, "cook_defocused", num_cook_defocused_runs)

        for result in discover_strike_results: writeResult(experiment_aggregate_file, result)
        for result in cook_results: writeResult(experiment_aggregate_file, result)
        for result in cook_defocused_results: writeResult(experiment_aggregate_file, result)

        return RunExperimentSrvResponse(True)

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return RunExperimentSrvResponse(False)

def run_experiment(experimentName, experiment_path, scenarioName, num_runs):
    scenario_path = generateRunResultsDir(experiment_path, scenarioName)
    scenario_aggregate_file = initResultCsvFile(scenario_path, str(scenarioName + '_results'), experiments_csv_header)
    scenario_results = []

    for i in range(num_runs):

        # Prepare for run
        run_name = 'run_' + str(i) # run_name = experimentName + '_' + str(i)
        run_results_dir = generateRunResultsDir(scenario_path, run_name, True)
        result_file = initResultCsvFile(run_results_dir, 'run_results', individual_run_csv_header)

        # Run it!
        result = BrainProxy(run_name, scenarioName, demo_mode)
        rospy.sleep(1)

        # Close out
        formatted_result = format_run_result(result)
        quantitative_result = quantify_run_result(scenarioName, run_name, result)
        writeResult(result_file, formatted_result)
        writeResult(scenario_aggregate_file, quantitative_result)
        compileResults(experiment_path, run_results_dir)
        scenario_results.append(quantitative_result)

    return scenario_results

def main():
    rospy.init_node("experiments_node")
    rospy.Service("experiments_srv", RunExperimentSrv, run_experiments)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()