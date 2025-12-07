import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('path_smoothing_control')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Use TurtleBot3's official Gazebo launch (which includes proper plugins)
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        ),
    )

    # Your control node
    path_smoothing_node = Node(
        package='path_smoothing_control',
        executable='path_smoothing_node',
        name='path_smoothing_node',
        output='screen',
        parameters=[params_file],
    )

    rviz_config = os.path.join(pkg_dir, 'config', 'path_smoothing.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        path_smoothing_node,
        rviz_node
    ])
