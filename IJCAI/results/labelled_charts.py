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

#########################################################################################
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
##### With Legend #######################################################################
def aggregate_episodes_vs_avg_trial_times_with_legend(scenario_data):
    COOK_concatenated_data = pd.DataFrame({'Number_of_Episodes' : scenario_data['cook']['num_trials'], 
                                           'Average_Action_Cost_perEpisode' : scenario_data['cook']['avg_trial_times'],
                                           'Standard_Deviations' : scenario_data['cook']['std_devs']})
    COOKDEFOCUSED_concatenated_data = pd.DataFrame({'Number_of_Episodes' : scenario_data['cook_defocused']['num_trials'], 
                                                    'Average_Action_Cost_perEpisode' : scenario_data['cook_defocused']['avg_trial_times'],
                                                    'Standard_Deviations' : scenario_data['cook_defocused']['std_devs']})
    DISCOVERSTRIKE_concatenated_data = pd.DataFrame({'Number_of_Episodes' : scenario_data['discover_strike']['num_trials'], 
                                                     'Average_Action_Cost_perEpisode' : scenario_data['discover_strike']['avg_trial_times'],
                                                     'Standard_Deviations' : scenario_data['discover_strike']['std_devs']})
    fig3, ax3 = plt.subplots()
    ax3.scatter(COOK_concatenated_data['Number_of_Episodes'],COOK_concatenated_data['Average_Action_Cost_perEpisode'], c='r', s=COOK_concatenated_data['Standard_Deviations'], alpha=0.5, label="Experiment 2")
    ax3.scatter(COOKDEFOCUSED_concatenated_data['Number_of_Episodes'],COOKDEFOCUSED_concatenated_data['Average_Action_Cost_perEpisode'], c='g', s=COOKDEFOCUSED_concatenated_data['Standard_Deviations'], alpha=0.5,  label="Experiment 3")
    ax3.scatter(DISCOVERSTRIKE_concatenated_data['Number_of_Episodes'],DISCOVERSTRIKE_concatenated_data['Average_Action_Cost_perEpisode'], c='b', s=DISCOVERSTRIKE_concatenated_data['Standard_Deviations'], alpha=0.5, label="Experiment 1")
    ax3.set_xlabel('# of Episodes', fontsize=15)
    ax3.set_ylabel('Avg Action Cost', fontsize=15)
    ax3.set_title('Average Action Cost vs. Number of Episodes')
    ax3.grid(True)
    ax3.legend()
    fig3.tight_layout()
    plt.show()

#########################################################################################
##### With Legend #######################################################################
def aggregate_T_vs_avg_trial_times_with_legend(scenario_data):
    COOK_concatenated_data = pd.DataFrame({'Number_of_Episodes' : scenario_data['cook']['num_trials'], 
                                           'Average_Action_Cost_perEpisode' : scenario_data['cook']['avg_trial_times'],
                                           'Standard_Deviations' : scenario_data['cook']['std_devs'], 
                                           'T' : scenario_data['cook']['T']})
    COOKDEFOCUSED_concatenated_data = pd.DataFrame({'Number_of_Episodes' : scenario_data['cook_defocused']['num_trials'], 
                                                    'Average_Action_Cost_perEpisode' : scenario_data['cook_defocused']['avg_trial_times'],
                                                    'Standard_Deviations' : scenario_data['cook_defocused']['std_devs'],
                                                    'T' : scenario_data['cook_defocused']['T']})
    DISCOVERSTRIKE_concatenated_data = pd.DataFrame({'Number_of_Episodes' : scenario_data['discover_strike']['num_trials'], 
                                                     'Average_Action_Cost_perEpisode' : scenario_data['discover_strike']['avg_trial_times'],
                                                     'Standard_Deviations' : scenario_data['discover_strike']['std_devs'],
                                                     'T' : scenario_data['discover_strike']['T']})

    concatenated_data = pd.DataFrame({'Number_of_Episodes' : scenario_data['aggregate']['num_trials'], 
                                      'Average_Action_Cost_perEpisode' : scenario_data['aggregate']['avg_trial_times'],
                                      'Standard_Deviations' : scenario_data['aggregate']['std_devs'], 
                                      'T' : scenario_data['aggregate']['T'],
                                      'Scenario' : scenario_data['aggregate']['scenario_colors']})
    fig4, ax4 = plt.subplots()
    ax4.scatter(COOK_concatenated_data['T'],COOK_concatenated_data['Average_Action_Cost_perEpisode'], c='r', s=COOK_concatenated_data['Standard_Deviations'], alpha=0.5, label="Experiment 2")
    ax4.scatter(COOKDEFOCUSED_concatenated_data['T'],COOKDEFOCUSED_concatenated_data['Average_Action_Cost_perEpisode'], c='g', s=COOKDEFOCUSED_concatenated_data['Standard_Deviations'], alpha=0.5,  label="Experiment 3")
    ax4.scatter(DISCOVERSTRIKE_concatenated_data['T'],DISCOVERSTRIKE_concatenated_data['Average_Action_Cost_perEpisode'], c='b', s=DISCOVERSTRIKE_concatenated_data['Standard_Deviations'], alpha=0.5, label="Experiment 1")
    ax4.set_xlabel('T value', fontsize=15)
    ax4.set_ylabel('Avg Action Cost', fontsize=15)
    ax4.set_title('Average Action Cost vs. T')
    ax4.grid(True)
    ax4.legend()
    fig4.tight_layout()
    plt.show()

    ###################################################################################################################
    # fig7, ax7 = plt.subplots()
    # ax7.set_title('All Experiments: Average Action Cost vs. T value')
    # # ax7.set_xlabel('# of Episodes', fontsize=15)
    # # ax7.set_ylabel('Avg Action Cost', fontsize=15)
    # ax7.grid(True)
    # ax7 = sns.boxplot(x="T", y="Average_Action_Cost_perEpisode", data=concatenated_data, hue="Scenario")
    # ax7 = sns.swarmplot(x="T", y="Average_Action_Cost_perEpisode", data=concatenated_data, color=".25", hue="Scenario")
    # plt.show()
#########################################################################################
#########################################################################################



data = generate_viz_data('aggregate.csv')
scenarios = ['aggregate']

aggregate_episodes_vs_avg_trial_times_with_legend(data)
aggregate_T_vs_avg_trial_times_with_legend(data)




