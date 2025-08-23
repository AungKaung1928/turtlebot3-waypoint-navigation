#!/usr/bin/env python3

"""
Complete Autonomous Navigation Launch File
=========================================

Launches everything needed for autonomous navigation:
1. Gazebo simulation with TurtleBot3
2. Nav2 navigation stack with SLAM
3. Waypoint navigator node

Author: Your Name
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate complete launch description for autonomous navigation."""
    
    # Get package directories
    autonomous_navigator_dir = get_package_share_directory('autonomous_navigator')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Gazebo world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            autonomous_navigator_dir,
            '/launch/gazebo_world_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Navigation launch (delayed to allow Gazebo to start)
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    autonomous_navigator_dir,
                    '/launch/navigation_launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items()
            )
        ]
    )
    
    # Waypoint navigator (delayed to allow Nav2 to initialize)
    waypoint_navigator_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='autonomous_navigator',
                executable='waypoint_navigator',
                name='waypoint_navigator',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        gazebo_launch,
        navigation_launch,
        waypoint_navigator_node,
    ])