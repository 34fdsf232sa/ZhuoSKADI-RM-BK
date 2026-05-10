#!/usr/bin/env python3
"""
Nav2 Mapless Navigation Launch File

This launches the complete Nav2 stack for mapless navigation:
- Local costmap only (no static map)
- Rolling window mode
- Uses L2 LiDAR point cloud for obstacle detection
- Fake odometry for testing (replace with real odom later)

Usage:
  ros2 launch mapless_nav nav2_mapless.launch.py
  
Prerequisites:
  - L2 LiDAR driver running: ros2 launch unitree_lidar_ros2 launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    mapless_nav_dir = get_package_share_directory('mapless_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    odom_mode = LaunchConfiguration('odom_mode')
    
    # Config file
    nav2_params_file = os.path.join(mapless_nav_dir, 'config', 'nav2_mapless_params.yaml')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    declare_odom_mode = DeclareLaunchArgument(
        'odom_mode',
        default_value='fake',
        choices=['fake', 'l2_imu', 'external'],
        description='Odometry mode: fake (testing), l2_imu (L2 IMU), external (FAST_LIO/slam_toolbox)'
    )

    # ==========================================================================
    # Odometry Node
    # ==========================================================================
    real_odom_node = Node(
        package='mapless_nav',
        executable='real_odom_node.py',
        name='real_odom_node',
        output='screen',
        parameters=[{
            'mode': odom_mode,
            'use_sim_time': use_sim_time,
        }]
    )
    
    # ==========================================================================
    # Static TF: base_link -> unilidar_lidar
    # ==========================================================================
    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_lidar',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'unilidar_lidar'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ==========================================================================
    # Nav2 Navigation Launch
    # ==========================================================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
        }.items()
    )
    
    # ==========================================================================
    # RViz with Nav2 config
    # ==========================================================================
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    # Delay Nav2 launch to ensure TF is ready
    delayed_nav2 = TimerAction(
        period=2.0,
        actions=[nav2_launch]
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_autostart,
        declare_use_rviz,
        declare_odom_mode,

        # Core TF and Odom
        real_odom_node,
        tf_base_to_lidar,
        
        # Nav2 (delayed)
        delayed_nav2,
        
        # Visualization
        rviz_node,
    ])
