# Model Predicitive Control on Turtlebot3 Burger on ROS2 Jazzy

## Implementation of Non Linear MPC on Turtlebot3 Burger as Multishooting Point Stabilization Problem using ROS2 Jazzy

Turtlebot3 is an open-source Differential Drive Robot used for educational purposes. This is an implementation of MPC on this differential drive robot using ROS2 Jazzy, Python and Casadi. This can help the turtlebot to move from its initial pose to the given goal pose by using robust control. If you're new to Turtlebot and Control Theory, trying to implement this project is a great starting step. This project gave me a good understanding: \
 --> Creating Nodes, Subscribers, Publishers using rclpy in ros2 jazzy \
 --> Using Casadi to define and solve the optimization problem \
 --> Non Linear Model Predictive Control \
 --> Writing ROS2 Scripts and Launch files \

 ## This is how it looks while its running:


 ## How to Use This:
 Assuming you already have a workspace, go to `~/workspace_ws/src/` \
 --> Clone the repository and go through the package.xml file to look into the requirements \
 --> Create A python virtual Environment using `python3 -m venv venv` in your `~/workspace_ws` and add `COLCON_IGNORE` in it, so that colcon  doesnt get disturbed while building \
 --> Install all the requirements using `pip install <PACKAGE_NAME>` \
 --> build the package being in workspace \
 --> Source your environment `source venv/bin/activate` \
 --> Source your install `source install/setup.bash` \
 --> 
