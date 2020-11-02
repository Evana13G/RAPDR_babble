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

5. PyperPlan (https://bitbucket.org/malte/pyperplan)
   a. Manually download the code to the src directory and rename it 'pyperplan'

6. Clone the RAPDR project

** At this point, the file heirarchy should take the following form:

        catkin_ws/
        catkin_ws/src/
        catkin_ws/src/RAPDR_babble/
        catkin_ws/src/*all baxter gazebo packages*
        catkin_ws/src/pyperplan/

8. Build:

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
        
#### DEVELOPMENT Run instructions
[FOR DEVELOPMENT MODE] Each of the following should be run in a separate terminal window:

1. Launch baxter sim:

        roslaunch baxter_gazebo baxter_world.launch

2. Spawn the data conversion module:

        rosrun environment scenario_data.py

3. Spawn the agent proxy:

        rosrun agent physical_agent_executor.py
        
4. Spawn the vision system proxy:

        rosrun agent vision.py

5. Spawn the environment elements:

        rosrun environment publish_environment.py
        rosrun environment load_environment.py

6. Spawn action primitive variation tool:

        rosrun action_primitive_variation APV_server.py

7. Spawn the PDDL node (each in a seperate terminal window):

        rosrun pddl planner.py

8. Spawn the knowledge base interface node:

            rosrun pddl knowledge_base.py
            
9. Run the agent reasoning node:

        rosrun agent brain.py
        
10. Call the brain service:

        rosservice call /brain_srv [tab complete for args]
  
To run testing files, replace steps 9 and 10 with the following:

        rosrun agent <test_file_name>
        rosservice call /test_service_name [tab complete for args]

Examples:

        rosrun agent test_actions.py   
        rosservice call /test_actions_srv [No Args]
        
        rosrun agent test_action_settings.py   
        rosservice call /test_action_settings_srv [No Args]
        
#### Other Info
The URDF models are inside the baxter_simulation package in a folder that I believe is called baxter_sim_examples/models. The URDF model for the table and the wall is called cafe_table. 

1. For proof of concept scenario #1, we assume the following protocol:

"left_gripper" - left gripper \n
"right_gripper" - right gripper \n
"cup" - cup \n
"cover" - cover \n

2. Many of the .py files might not be runnable. Navigate to their directories and run:

        chmod +x [filename]
        
For example:

        chmod +x brain.py
        
