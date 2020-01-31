# RAPDR_babble
Research in Action Primitive Discovery in Robotics through Parameter Variation

#### Setup Instructions
Setup each of the following, in order:

1. Ubuntu 16.04

2. ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu & http://wiki.ros.org/ROS/Tutorials)

3. Gazebo 7 (Installs with ROS installation) 

4. Baxter Gazebo Simulation (http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)
    a. Note, we are not using the `baxter.sh sim` script

5. PyperPlan (https://bitbucket.org/malte/pyperplan)
   a. Manually download the code to the src directory and rename it 'pyperplan'

6. Clone the RAPDR project

** At this point, the file heirarchy should take the following form:

        catkin_ws/
        catkin_ws/src/
        catkin_ws/src/RAPDR/
        catkin_ws/src/*all baxter gazebo packages*
        catkin_ws/src/pyperplan/

8. Build:

        cd ~/catkin_ws
        catkin_make
        source devel/setup.bash

#### Run instructions
Each of the following should be run in a separate terminal window:

1. Launch baxter sim. 

        roslaunch baxter_gazebo baxter_world.launch

2. Spawn the agent

        rosrun agent physical_agent_executor.py
        
3. Spawn the environment elements (table, buttons, object)

        rosrun environment initialize_environment.py

4. Spawn data conversion node (converts raw data into predicate form)

        rosrun environment scenario_data.py

5. Spawn the PDDL nodes (each in a seperate terminal window):

    1. Service for generating pddl plans

            rosrun pddl plan_generator.py

    2. Service for executing pddl plans

            rosrun pddl plan_executor.py

6. Run the agent brain (creepy). All configs are set here.

        rosrun agent brain.py


#### Other Info
The URDF models are inside the baxter_simulation package in a folder that I believe is called baxter_sim_examples/models. The URDF model for the table and the wall is called cafe_table. 

1. For proof of concept scenario #1, we assume the following protocol:

"left_gripper" - left gripper \n
"right_gripper" - right gripper \n
"block" - object to obtain \n
"left_button" - left button \n
"right_button" - right button \n

2. Many of the .py files might not be runnable. Navigate to their directories and run:

        chmod +x [filename]
        
For example:

        chmod +x brain.py
        
