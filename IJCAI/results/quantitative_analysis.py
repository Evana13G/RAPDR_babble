import matplotlib.pyplot as plt
import numpy as np
import csv
import seaborn as sns
import pandas as pd


all_scenario_data = {'cook' : {'total_times' : [],
                           'T' : [],
                           'num_trials' : [],
                           'avg_trial_times' : [],
                           'success_actions' : [],
                           'std_devs' : []},
      'discover_strike' : {'total_times' : [],
                           'T' : [],
                           'num_trials' : [],
                           'avg_trial_times' : [],
                           'success_actions' : [],
                           'std_devs' : []},
       'cook_defocused' : {'total_times' : [],
                           'T' : [],
                           'num_trials' : [],
                           'avg_trial_times' : [],
                           'success_actions' : [],
                           'std_devs' : []},
            'aggregate' : {'total_times' : [],
                           'T' : [],
                           'num_trials' : [],
                           'avg_trial_times' : [],
                           'success_actions' : [],
                           'std_devs' : [],
                           'scenarios' : []}}

def generate_viz_data(full_filepath):
    # Load Data
    raw_data = []
    with open(full_filepath) as csvfile:
        rdr = csv.reader(csvfile, delimiter=' ')
        for row in rdr:
            raw_data.append(row)

    for datum in raw_data[1:]:
        scenario = datum[0]
        T = int(datum[2])
        total_times = float(datum[3])
        num_trails = float(datum[4])
        avg_trial_time = float(datum[5])
        success_actions = datum[6][2:-2]
        std_devs = float(datum[8])
        all_scenario_data[scenario]['total_times'].append(total_times)
        all_scenario_data[scenario]['T'].append(T)
        all_scenario_data[scenario]['num_trials'].append(num_trails)
        all_scenario_data[scenario]['avg_trial_times'].append(avg_trial_time)
        all_scenario_data[scenario]['success_actions'].append(success_actions)
        all_scenario_data[scenario]['std_devs'].append(std_devs)

        all_scenario_data['aggregate']['scenarios'].append(scenario)
        all_scenario_data['aggregate']['total_times'].append(total_times)
        all_scenario_data['aggregate']['T'].append(T)
        all_scenario_data['aggregate']['num_trials'].append(num_trails)
        all_scenario_data['aggregate']['avg_trial_times'].append(avg_trial_time)
        all_scenario_data['aggregate']['success_actions'].append(success_actions)
        all_scenario_data['aggregate']['std_devs'].append(std_devs)

    return all_scenario_data

#########################################################################################
##### ALL CHARTS ########################################################################
def gen_results(scenario_data, sample): 
    total_times = np.array(scenario_data[sample]['total_times'])
    T =  np.array(scenario_data[sample]['T'])
    num_trials = np.array(scenario_data[sample]['num_trials'])
    avg_trial_times = np.array(scenario_data[sample]['avg_trial_times'])
    success_actions = np.array(scenario_data[sample]['success_actions'])
    std_devs = np.array(scenario_data[sample]['std_devs'])

    avg_action_cost = np.mean(total_times)
    avg_num_episodes = np.mean(num_trials)

    return avg_action_cost, avg_num_episodes


data = generate_viz_data('aggregate.csv')
scenarios = ['discover_strike', 'cook', 'cook_defocused', 'aggregate']

for scenario in scenarios:
    aac, ane = gen_results(data, scenario)
    print('Scenario: ' + str(scenario))
    print('--- Average Action Cost: ' + str(aac))
    print('--- Average # Episodes: ' + str(ane))


