import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('path_smoothing_control')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'path_smoothing.rviz')

    return LaunchDescription([
        Node(
            package='path_smoothing_control',
            executable='path_smoothing_node',
            name='path_smoothing_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='path_smoothing_control',
            executable='mock_sim_node.py',
            name='mock_sim_node',
            output='screen',
            parameters=[{'start_x': 0.0, 'start_y': 0.0, 'start_theta': 0.0}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
