# Model Predicitive Control on Turtlebot3 Burger on ROS2 Jazzy

## Implementation of Non Linear MPC on Turtlebot3 Burger as Multishooting Point Stabilization Problem using ROS2 Jazzy

Turtlebot3 is an open-source Differential Drive Robot used for educational purposes. This is an implementation of MPC on this differential drive robot using ROS2 Jazzy, Python and Casadi. This can help the turtlebot to move from its initial pose to the given goal pose by using robust control. If you're new to Turtlebot and Control Theory, trying to implement this project is a great starting step. This project gave me a good understanding: 
 - Creating Nodes, Subscribers, Publishers using rclpy in ros2 jazzy 
 - Using Casadi to define and solve the optimization problem 
 - Non Linear Model Predictive Control 
 - Writing ROS2 Scripts and Launch files 

 ## This is how it looks while its running:


 ## How to Use This:
 Assuming you already have a workspace and also have some basic knowledge about launch and run scripts, go to `~/workspace_ws/src/` 
  ### Basic Steps:
  - Clone the repository and go through the package.xml file to look into the requirements 
  - Create A python virtual Environment using `python3 -m venv venv` in your `~/workspace_ws` and add   `touch COLCON_IGNORE` in it
  - Source your environment `source venv/bin/activate` 
  - Install all the requirements using `pip install <PACKAGE_NAME>` 
  - build the package being in workspace `colcon build --symlink-true`
  - Source your build `source install/setup.bash` 



#### 1) Using RUN Scripts
SSH to your phsyical turtlebot and launch bringup, or launch gazebo simulation. \
Now there are some ways to run this, but in any step, there are two main scripts in the package. 

1) "bot_mpc.py" : This script will be able to take in messages from /odom and /pose, process them using mpc and find the optimal control input and publish it to /cmd_vel. You can change the parameters dynamically by simply typing using `rqt` in another terminal and plugins-> dynamic reconfigure. \
        you can use this script by `ros2 run <your_pacakge_name> bot_mpc` 
 
2)  "turtlebot3_mpc.py" : This script has the same functions as bot_mpc.py along with them, it will also publish the running position and control costs of MPC, you can similarly view them in `rqt`, plugins->plots. \
        you can use this script by `ros2 run <your_pacakge_name> mpc` 

#### 2) Using Launch scripts
Launch scripts are directly available in this package under the launch directory, the three main components of these are: 
1) launching Gazebo
2) launching pre-configured RVIZ2 
3) Running MPC Node 

There are three launch files included: 
1) "mpc.launch.py": launches rviz2 and runs bot_mpc.py
2) "run_mpc.launch.py": launches rviz2, gazebo simulation in empty_world and runs bot_mpc.py
3) "mopc.launch.py": launches rviz2, gazebo simulation in empty_world and runs turtlebot3_mpc.py

Depending on the requirement, use the appropriate launch file or scripts.

## Updates:

This is a very simple implementation of MPC. I will try to add more features such as obstacle avoidance and multiple robot running scripts. 

#### If You have any suggestions, want to contribute or have any cool project ideas to work on, do contact me





