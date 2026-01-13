import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mpc_pkg_path = get_package_share_directory('turtlebot3_mpc')
    rviz_config_path = os.path.join(mpc_pkg_path, 'rviz', 'mpc_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    mpc_node = Node(
        package='turtlebot3_mpc',
        executable='bot_mpc',
        name='mpc_controller',
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        mpc_node
    ])