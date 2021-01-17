import matplotlib.pyplot as plt
import numpy as np
import csv
import seaborn as sns
import pandas as pd

def generate_RAPDR_babble_viz(full_filepath):

    scenario_data = {'cook' : {'total_times' : [],
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
        scenario_data[scenario]['total_times'].append(total_times)
        scenario_data[scenario]['T'].append(T)
        scenario_data[scenario]['num_trials'].append(num_trails)
        scenario_data[scenario]['avg_trial_times'].append(avg_trial_time)
        scenario_data[scenario]['success_actions'].append(success_actions)
        scenario_data[scenario]['std_devs'].append(std_devs)

        scenario_data['aggregate']['scenarios'].append(scenario)
        scenario_data['aggregate']['total_times'].append(total_times)
        scenario_data['aggregate']['T'].append(T)
        scenario_data['aggregate']['num_trials'].append(num_trails)
        scenario_data['aggregate']['avg_trial_times'].append(avg_trial_time)
        scenario_data['aggregate']['success_actions'].append(success_actions)
        scenario_data['aggregate']['std_devs'].append(std_devs)

    scenario_data['cook']['colors'] = [color_legend[x] for x in scenario_data['cook']['success_actions']]
    scenario_data['cook_defocused']['colors'] = [color_legend[x] for x in scenario_data['cook_defocused']['success_actions']]
    scenario_data['discover_strike']['colors'] = [color_legend[x] for x in scenario_data['discover_strike']['success_actions']]
    scenario_data['aggregate']['colors'] = [color_legend[x] for x in scenario_data['aggregate']['success_actions']]

    # scenario_data['aggregate']['colors'] = [scenario_color_legend[x] for x in scenario_data['aggregate']['scenarios']]

    scenarios = ['discover_strike', 'cook', 'cook_defocused', 'aggregate']
    for i in range(len(scenarios)):  
        sample = scenarios[i]           
        T =  scenario_data[sample]['T']
        num_trials = scenario_data[sample]['num_trials']
        avg_trial_times = scenario_data[sample]['avg_trial_times']
        colors = scenario_data[sample]['colors']
        std_devs = scenario_data[sample]['std_devs']

        concatenated_data = pd.DataFrame({'Number_of_Episodes' : num_trials, 'Average_Action_Cost_perEpisode' : avg_trial_times})
        
        fig, ax = plt.subplots()
        ax.scatter(num_trials,avg_trial_times, c=colors, s=std_devs, alpha=0.5)
        ax.set_xlabel('# of Episodes', fontsize=15)
        ax.set_ylabel('Avg Action Cost', fontsize=15)
        if sample == 'aggregate':
            ax.set_title('All Experiments: Number of Episodes vs. Average Action Cost')
        else:
            ax.set_title('Experiment ' + str(i+1) + ': Number of Episodes vs. Average Action Cost')
        ax.grid(True)
        fig.tight_layout()
        plt.show()

        fig2, ax2 = plt.subplots()
        if sample == 'aggregate':
            ax2.set_title('All Experiments: Number of Episodes vs. Average Action Cost')
        else:
            ax2.set_title('Experiment ' + str(i+1) + ': Number of Episodes vs. Average Action Cost')
        ax2.grid(True)
        ax2 = sns.boxplot(x="Number_of_Episodes", y="Average_Action_Cost_perEpisode", data=concatenated_data)
        ax2 = sns.swarmplot(x="Number_of_Episodes", y="Average_Action_Cost_perEpisode", data=concatenated_data, color=".25")
        plt.show()

#########################################################################################
#########################################################################################


#########################################################################################
#########################################################################################
    COOK_concatenated_data = pd.DataFrame({'Number_of_Episodes' : scenario_data['cook']['num_trials'], 
                                           'Average_Action_Cost_perEpisode' : scenario_data['cook']['avg_trial_times'],
                                           'Standard_Deviations' : scenario_data['cook']['std_devs']})
    COOKDEFOCUSED_concatenated_data = pd.DataFrame({'Number_of_Episodes' : scenario_data['cook_defocused']['num_trials'], 
                                                    'Average_Action_Cost_perEpisode' : scenario_data['cook_defocused']['avg_trial_times'],
                                                    'Standard_Deviations' : scenario_data['cook_defocused']['std_devs']})
    DISCOVERSTRIKE_concatenated_data = pd.DataFrame({'Number_of_Episodes' : scenario_data['discover_strike']['num_trials'], 
                                                     'Average_Action_Cost_perEpisode' : scenario_data['discover_strike']['avg_trial_times'],
                                                     'Standard_Deviations' : scenario_data['discover_strike']['std_devs']})
    fig, ax = plt.subplots()
    ax.scatter(COOK_concatenated_data['Number_of_Episodes'],COOK_concatenated_data['Average_Action_Cost_perEpisode'], c='r', s=COOK_concatenated_data['Standard_Deviations'], alpha=0.5, label="Experiment 2")
    ax.scatter(COOKDEFOCUSED_concatenated_data['Number_of_Episodes'],COOKDEFOCUSED_concatenated_data['Average_Action_Cost_perEpisode'], c='g', s=COOKDEFOCUSED_concatenated_data['Standard_Deviations'], alpha=0.5,  label="Experiment 3")
    ax.scatter(DISCOVERSTRIKE_concatenated_data['Number_of_Episodes'],DISCOVERSTRIKE_concatenated_data['Average_Action_Cost_perEpisode'], c='b', s=DISCOVERSTRIKE_concatenated_data['Standard_Deviations'], alpha=0.5, label="Experiment 1")
    ax.set_xlabel('# of Episodes', fontsize=15)
    ax.set_ylabel('Avg Action Cost', fontsize=15)
    ax.set_title('Number of Episodes vs. Average Action Cost')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    plt.show()
#########################################################################################
#########################################################################################

generate_RAPDR_babble_viz('aggregate.csv')






