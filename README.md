# RAPDR_param
Research in Action Primitive Discovery in Robotics through Parameter Variation

#### Setup Instructions
Setup each of the following, in order:

1. Ubuntu 16.04

2. ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu & http://wiki.ros.org/ROS/Tutorials)

3. Gazebo 7 (Installs with ROS installation) 

4. MoveIt! (http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html?fbclid=IwAR0RQ0jpDFDdohrVlnNNGAaPGvqAPEDsAIXnmBv2ROWGV8wfIdpRrIF9W-8)

5. UR5 (https://github.com/gtatiya/Hacking-SotA-UR5.git)

6. PyperPlan (https://bitbucket.org/malte/pyperplan)
   a. Manually download the code to the src directory and rename it 'pyperplan'

7. Clone the RAPDR_param project

** At this point, the file heirarchy should take the following form:

        catkin_ws/src/Hacking-SotA-UR5/
        catkin_ws/src/moveit_robots
        catkin_ws/src/pyperplan/
        catkin_ws/src/RAPDR_param/

8. Build:

        cd ~/catkin_ws
        catkin_make
        source devel/setup.bash

#### Run instructions
Each of the following should be run in a separate terminal window:

1. Launch UR5 sim. 

        roslaunch agent UR5.launch

2. Spawn the physical agent executor node, responsible for advertising UR5 action services. 

        rosrun agent physical_agent_executor.py
        
3. Spawn the environment elements (cup and cover)

        rosrun environment initialize_environment.py

4. Spawn data push_object action service node 

        rosrun agent push_object.py

5. Spawn data grasp_object action service node 

        rosrun agent grasp_object.py

#### Other Info

1. Many of the .py files might not be runnable. Navigate to their directories and run:

        chmod +x [filename]
        
For example:

        chmod +x brain.py
        
        


 
