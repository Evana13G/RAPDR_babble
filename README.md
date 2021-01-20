# RAPDR_babble
Research in Action Primitive Discovery in Robotics through Parameter Variation

#### Setup Instructions
Setup each of the following, in order:

1. Ubuntu 16.04

2. ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu & http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
*SKIP the line 'sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential*

3. Gazebo 7 (Installs with ROS installation) 

4. Baxter Gazebo Simulation (http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)
    a. Note, we are not using the `baxter.sh sim` script

5. Unzip the RAPDR project
        
6. Run the setup.sh script in RAPDR_babble (Only need to run once after installation)

        cd RAPDR_babble && ./setup.sh
        
7. Build:

        cd ~/catkin_ws
        catkin_make
        source devel/setup.bash

#### Run instructions

To run the code, use the following:

Each of the following should be run in a separate terminal window:

1. Launch baxter sim:

        roslaunch baxter_gazebo baxter_world.launch

2. Spawn RAPDR Nodes:

        roslaunch agent RAPDR.launch
        
3. Run an experiment:

        rosservice call /experiments_srv [tab complete for args]
            "experiment_name: 'experiment'
             demo_mode: false
             num_discover_strike_runs: 1
             num_cook_runs: 0" 

Name the experiment (experiment_name), which RAPDR will use to name the results in the results directory after running. Specify if you would like a demo of the agent accomplishing its goal in the 'original' scenario (demo_mode : true), otherwise the agent will enter the 'novel' scenerio. Specify the number of runs of each type of experiment you would like to run. 

4. To run a test script: 

        rosservice call /[test name] [tab complete for args]

Examples:

        rosservice call /test_actions_srv "{}" [No Args]
        
        rosservice call /test_action_settings_srv "{}" [No Args]
        

For more instructions on writing/running test nodes, see below which will redirect to the developers wiki. 


#### Other Info <br />

1. Many of the .py files might not be runnable. Navigate to their directories and run:

        chmod +x [filename]
        
For example:

        chmod +x brain.py
        
