import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Configuration Variables
    # Change 'burger' to 'waffle' or 'waffle_pi' if needed
    turtlebot_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # Path to your RViz config (Create this file first, see instructions below)
    # If you don't have one yet, we can temporarily omit the '-d' argument or point to a default
    mpc_pkg_path = get_package_share_directory('turtlebot3_mpc')
    rviz_config_path = os.path.join(mpc_pkg_path, 'rviz', 'mpc_view.rviz')

    # 2. Set Environment Variables (Ensure model is set for Gazebo)
    set_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot_model)

    # 3. Include TurtleBot3 Gazebo Empty World Launch
    # This matches your request: "launch turtlebot3_gazebo package empty_world.launch.py"
    tb3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            )
        )
    )

    # 4. Define RViz2 Node
    # Loads with the specific config file to show Odom and Pose Goal
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 5. Define Your Custom MPC Node
    # Matches your request: "turtlebot3_mpc package mpc"
    mpc_node = Node(
        package='turtlebot3_mpc',
        executable='mpc',
        name='mpc_controller',
        output='screen'
    )

    return LaunchDescription([
        set_model_env,
        tb3_gazebo_launch,
        rviz_node,
        mpc_node
    ])