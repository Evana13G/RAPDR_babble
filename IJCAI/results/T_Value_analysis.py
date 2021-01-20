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
               'push-movementmagnitude:0.266666666667' : 'g',
               'push-movementmagnitude:0.1' : 'b',
               'push-rate:27.5' : 'y',
               'push-rate:39.75' : 'w',
               'push-movementmagnitude:0.225' : 'k',
               'push-rate:76.5' : 'c',
               'push-rate:150.0' : 'm',
               'shake-movementmagnitude:5.0' : '#33A6FF',
               'shake-rate:10.5' : '#FF813C', 
               'shake-rate:20.0' : '#CCCF9F', 
               'shake-orientation:top' : '#CACACA',
               'push-orientation:right' : '#944AF9',
               'push-orientation:top' : '#4F7A3C', 
               'push-movementmagnitude:0.6' : '#4902FF',
               'push-rate:113.25' : '#FFC602',
               'push-movementmagnitude:0.475' : '#82EAF3', 
               'push-movementmagnitude:0.516666666667' : '#E58AC2', 
               'push-movementmagnitude:0.183333333333' : '#78616F', 
               'push-rate:125.5' : '#04FEBA'}

scenario_color_legend = {'discover_strike' : 'r',
               'cook' : 'g',
               'cook_defocused' : 'b'}

experiment_name_legend = {'discover_strike' : '1',
                          'cook' : '2',
                          'cook_defocused' : '3'}

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

    all_scenario_data['cook']['colors'] = [color_legend[x] for x in all_scenario_data['cook']['success_actions']]
    all_scenario_data['cook_defocused']['colors'] = [color_legend[x] for x in all_scenario_data['cook_defocused']['success_actions']]
    all_scenario_data['discover_strike']['colors'] = [color_legend[x] for x in all_scenario_data['discover_strike']['success_actions']]
    all_scenario_data['aggregate']['colors'] = [color_legend[x] for x in all_scenario_data['aggregate']['success_actions']]
    all_scenario_data['aggregate']['scenario_colors'] = [scenario_color_legend[x] for x in all_scenario_data['aggregate']['scenarios']]

    return all_scenario_data


#########################################################################################
def episodes_vs_T(scenario_data, scenarios):
    T = []
    num_trials = []

    for i in range(len(scenarios)):  
        sample = scenarios[i]           
        T.extend(scenario_data[sample]['T'])
        num_trials.extend(scenario_data[sample]['num_trials'])

    concatenated_data = pd.DataFrame({'T' : T, 'Number_of_Episodes' : num_trials})

    figure, boxplot = plt.subplots()
    boxplot.set_title('Experiment 1 & 2: Number of Episodes vs. T value',  fontsize=15)
    boxplot.set_xlabel('T Value', fontsize=15)
    boxplot.set_ylabel('# of Episodes', fontsize=15)
    boxplot.grid(True)
    boxplot = sns.boxplot(x="T", y="Number_of_Episodes", data=concatenated_data)
    boxplot = sns.swarmplot(x="T", y="Number_of_Episodes", data=concatenated_data, color=".25")
    plt.show()

#########################################################################################
def avg_trial_times_vs_T(scenario_data, scenarios):
    T = []
    avg_trial_times = []

    for i in range(len(scenarios)):  
        sample = scenarios[i]           
        T.extend(scenario_data[sample]['T'])
        avg_trial_times.extend(scenario_data[sample]['avg_trial_times'])

    concatenated_data = pd.DataFrame({'T' : T, 'Average_Action_Cost_perEpisode' : avg_trial_times})

    figure, boxplot = plt.subplots()
    boxplot.set_title('Experiment 1 & 2: Average Action Cost vs. T value',  fontsize=15)
    boxplot.set_xlabel('T Value', fontsize=15)
    boxplot.set_ylabel('Avg Action Cost', fontsize=15)
    boxplot.grid(True)
    boxplot = sns.boxplot(x="T", y="Average_Action_Cost_perEpisode", data=concatenated_data)
    boxplot = sns.swarmplot(x="T", y="Average_Ascenariosction_Cost_perEpisode", data=concatenated_data, color=".25")
    plt.show()

#########################################################################################
def total_time_vs_T(scenario_data, scenarios):
    colors = []
    T = []
    total_times = []

    for i in range(len(scenarios)):  
        sample = scenarios[i]
        colors.extend(experiment_name_legend[sample]*len(scenario_data[sample]['T']))
        T.extend(scenario_data[sample]['T'])
        total_times.extend(scenario_data[sample]['total_times'])

    concatenated_data = pd.DataFrame({'T_Value' : T, 'Total_Action_Cost' : total_times, 'Scenarios' : colors})

    figure, boxplot = plt.subplots()
    boxplot.set_title('Experiment 1 & 2: Total Action Cost vs. T value',  fontsize=30)
    boxplot.set_xlabel('T Value', fontsize=30)
    boxplot.set_ylabel('Total Action Cost', fontsize=30)
    boxplot.grid(True)

    boxplot = sns.boxplot(x="T_Value", y="Total_Action_Cost", data=concatenated_data, hue="Scenarios")
    boxplot = sns.swarmplot(x="T_Value", y="Total_Action_Cost", data=concatenated_data, hue="Scenarios", color='.25',dodge=True)
    plt.show()


data = generate_viz_data('aggregate.csv')
scenarios = ['discover_strike', 'cook', 'cook_defocused', 'aggregate']
# avg_trial_times_vs_T(data, ['discover_strike', 'cook'])
# episodes_vs_T(data, ['discover_strike', 'cook'])
total_time_vs_T(data, ['discover_strike', 'cook'])






























