#!/usr/bin/env python3
"""
Nav2 Navigation Launch File for TurtleBot3
Launches Nav2 navigation stack with SLAM and localization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_navigation2_dir = get_package_share_directory('turtlebot3_navigation2')
    autonomous_navigator_dir = get_package_share_directory('autonomous_navigator')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    # Launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether to run SLAM'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(turtlebot3_navigation2_dir, 'map', 'map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(autonomous_navigator_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    # SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch'),
            '/slam_launch.py'
        ]),
        condition=IfCondition(slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )
    
    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch'),
            '/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_yaml_file,
        }.items()
    )
    
    # Behavior server for recovery behaviors
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )
    
    # RViz with custom config
    rviz_config_file = os.path.join(autonomous_navigator_dir, 'rviz', 'navigation.rviz')
    
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch'),
            '/rviz_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        
        slam_launch,
        navigation_launch,
        behavior_server,
        rviz_launch,
    ])