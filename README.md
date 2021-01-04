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

5. Clone the RAPDR project

        git clone https://github.com/Evana13G/RAPDR_babble.git
        
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
        
3. Call the brain service:

        rosservice call /brain_srv [tab complete for args]

  
To run test scripts:

        rosservice call /test_service_name [tab complete for args]

Examples:

        rosservice call /test_actions_srv [No Args]
        
        rosservice call /test_action_settings_srv [No Args]
        
For more instructions on writing/running test nodes, see below which will redirect to the developers wiki. 

#### DEVELOPMENT Run instructions <br />
[FOR DEVELOPMENT MODE] See https://github.com/Evana13G/RAPDR_babble/wiki/Developers-Instructions

#### Other Info <br />
The URDF models are inside the baxter_simulation package in a folder that I believe is called baxter_sim_examples/models. The URDF model for the table and the wall is called cafe_table. 

1. For proof of concept scenario #1, we assume the following protocol:

"left_gripper" - left gripper <br />
"right_gripper" - right gripper <br />
"cup" - cup <br />
"cover" - cover <br />

2. Many of the .py files might not be runnable. Navigate to their directories and run:

        chmod +x [filename]
        
For example:

        chmod +x brain.py
        
