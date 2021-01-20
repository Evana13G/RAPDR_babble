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

color_legend = {'push-rate:3.0' : 'r',
               'push-MM:0.267' : 'g',
               'push-MM:0.1' : 'b',
               'push-rate:27.5' : 'y',
               'push-rate:39.75' : '#DAF7A6',
               'push-MM:0.225' : 'k',
               'push-rate:76.5' : 'c',
               'push-rate:150.0' : 'm',
               'shake-MM:5.0' : '#33A6FF',
               'shake-rate:10.5' : '#FF813C', 
               'shake-rate:20.0' : '#CCCF9F', 
               'shake-orientation:top' : '#CACACA',
               'push-orientation:right' : '#944AF9',
               'push-orientation:top' : '#4F7A3C', 
               'push-MM:0.6' : '#4902FF',
               'push-rate:113.25' : '#FFC602',
               'push-MM:0.475' : '#82EAF3', 
               'push-MM:0.517' : '#E58AC2', 
               'push-MM:0.183' : '#78616F', 
               'push-rate:125.5' : '#04FEBA'}

scenario_color_legend = {'discover_strike' : 'r',
               'cook' : 'g',
               'cook_defocused' : 'b'}

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
        action_name = datum[6][2:-2].split(':')
        action_name[0] = action_name[0].replace('movementmagnitude', 'MM')
        if action_name[1] not in ['top', 'left', 'right']:
            success_actions = action_name[0] + ':' + str(round(float(action_name[1]), 3))
        else:
            success_actions = action_name[0] + ':' + action_name[1]
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

    all_scenario_data['cook']['colors'] = [color_legend[x] for x in all_scenario_data['cook']['success_actions']]
    all_scenario_data['cook_defocused']['colors'] = [color_legend[x] for x in all_scenario_data['cook_defocused']['success_actions']]
    all_scenario_data['discover_strike']['colors'] = [color_legend[x] for x in all_scenario_data['discover_strike']['success_actions']]
    all_scenario_data['aggregate']['colors'] = [color_legend[x] for x in all_scenario_data['aggregate']['success_actions']]

    all_scenario_data['aggregate']['scenario_colors'] = [scenario_color_legend[x] for x in all_scenario_data['aggregate']['scenarios']]

    return all_scenario_data

#########################################################################################
##### ALL CHARTS ########################################################################
def success_actions(scenario_data, experiment,  experiment_name=''):


    total_times = scenario_data[experiment]['total_times']
    T =  scenario_data[experiment]['T']
    num_trials = scenario_data[experiment]['num_trials']
    avg_trial_times = scenario_data[experiment]['avg_trial_times']
    success_actions = scenario_data[experiment]['success_actions']
    std_devs = scenario_data[experiment]['std_devs']
    success_action_colors = scenario_data[experiment]['colors']


    plt.rcdefaults()
    fig, ax = plt.subplots()

    # Example data

    actions = tuple(list(set(success_actions)))
    y_pos = np.arange(len(actions))
    scenarios = {}

    for entry in actions:
        scenarios[entry] = 0

    for entry in success_actions:
        scenarios[entry] += 1


    num_scenarios = []
    for a in actions:
        num_scenarios.append(scenarios[a])

    sorted_data = [(actions[i], num_scenarios[i]) for i in range(len(actions))]
    sorted_data.sort(key = lambda x: x[1], reverse=True) 
    actions = [x[0] for x in sorted_data]
    num_scenarios = [x[1] for x in sorted_data]
    colors = [color_legend[x] for x in actions]
    ax.barh(y_pos, num_scenarios, align='center', color='c')
    ax.set_yticks(y_pos)
    ax.set_yticklabels(actions, fontsize=15)
    ax.invert_yaxis()  # labels read top-to-bottom
    ax.set_xlabel('# of Trials', fontsize=15)
    if experiment_name != '':
        experiment_name = experiment_name + ': '
    ax.set_title(str(experiment_name) + '# of Trials per Success Action', fontsize=25)

    plt.show()




#########################################################################################

data = generate_viz_data('aggregate.csv')
# scenarios = ['discover_strike', 'cook', 'cook_defocused', 'aggregate']
scenarios = ['aggregate']

success_actions(data, 'discover_strike', experiment_name='Experiment 1')
success_actions(data, 'cook',  experiment_name='Experiment 2')
success_actions(data, 'cook_defocused',  experiment_name='Experiment 3')

