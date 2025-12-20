#!/usr/bin/env python3
"""
Full Navigation Launch - Single Command
Replicates exact 3-terminal workflow
"""
import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_navigation2_dir = get_package_share_directory('turtlebot3_navigation2')
    autonomous_navigator_dir = get_package_share_directory('autonomous_navigator')
    
    # Configuration files
    params_file = os.path.join(autonomous_navigator_dir, 'config', 'nav2_params.yaml')
    
    # 1. Launch Gazebo (Terminal 1 equivalent)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )
    
    # 2. Launch TurtleBot3 Navigation2 with SLAM (Terminal 2 equivalent)
    # 8 second delay for Gazebo to stabilize
    navigation = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(turtlebot3_navigation2_dir, 'launch', 'navigation2.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'slam': 'True',
                    'params_file': params_file,
                }.items()
            )
        ]
    )
    
    # 3. Launch Waypoint Navigator (Terminal 3 equivalent)
    # 25 second delay to wait for "Managed nodes are active"
    waypoint_navigator = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='autonomous_navigator',
                executable='waypoint_navigator',
                name='waypoint_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}],
                emulate_tty=True,
            )
        ]
    )
    
    return LaunchDescription([
        gazebo,
        navigation,
        waypoint_navigator,
    ])