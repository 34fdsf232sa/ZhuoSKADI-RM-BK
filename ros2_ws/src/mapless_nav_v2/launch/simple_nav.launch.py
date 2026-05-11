#!/usr/bin/env python3
"""
Simple Navigation Launch — COD Architecture: Single-Goal Mode

Mirrors cod_bringup/singlenav_launch.py pattern:
  1. Sensor processing
  2. Odometry
  3. Nav2 bringup
  4. Return-home node (primary navigation interface)

For single-goal navigation (return to home, go to point).
Use the /return_home service or send a NavigateToPose action goal directly.

Usage:
  ros2 launch mapless_nav_v2 simple_nav.launch.py
  ros2 launch mapless_nav_v2 simple_nav.launch.py odom_mode:=external
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
    v2_share = get_package_share_directory('mapless_nav_v2')
    v1_share = get_package_share_directory('mapless_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_mode = LaunchConfiguration('odom_mode')
    enable_rviz = LaunchConfiguration('enable_rviz')

    nav2_params = os.path.join(v2_share, 'config', 'nav2_params.yaml')
    fusion_params = os.path.join(v1_share, 'config', 'fusion_params.yaml')
    scan_params = os.path.join(v1_share, 'config', 'scan_conversion_params.yaml')
    return_home_params = os.path.join(v1_share, 'config', 'return_home_params.yaml')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_odom_mode = DeclareLaunchArgument(
        'odom_mode', default_value='fake',
        choices=['fake', 'l2_imu', 'external'])
    declare_enable_rviz = DeclareLaunchArgument('enable_rviz', default_value='true')

    # Odometry
    odom_node = Node(
        package='mapless_nav', executable='real_odom_node.py',
        name='real_odom_node', output='screen',
        parameters=[{'mode': odom_mode, 'use_sim_time': use_sim_time}])

    # TF
    tf_base_to_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_to_lidar',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'unilidar_lidar'])

    # Sensors
    fusion_node = Node(
        package='mapless_nav', executable='pointcloud_fusion_node',
        name='pointcloud_fusion_node', output='screen',
        parameters=[fusion_params, {'use_sim_time': use_sim_time}])

    scan_node = Node(
        package='mapless_nav', executable='pointcloud_to_scan_node.py',
        name='pointcloud_to_scan_node', output='screen',
        parameters=[scan_params, {'use_sim_time': use_sim_time}])

    # Nav2
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(v2_share, 'launch', 'bringup.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items())

    # Return Home (primary interface for single-goal nav)
    return_home_node = Node(
        package='mapless_nav', executable='return_home_node.py',
        name='return_home_node', output='screen',
        parameters=[return_home_params, {'use_sim_time': use_sim_time}])

    # RViz
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(v2_share, 'rviz', 'simple_nav.rviz')],
        condition=IfCondition(enable_rviz))

    delayed_nav2 = TimerAction(period=2.0, actions=[nav2_bringup])

    return LaunchDescription([
        declare_use_sim_time,
        declare_odom_mode,
        declare_enable_rviz,

        odom_node,
        tf_base_to_lidar,
        fusion_node,
        scan_node,
        delayed_nav2,
        return_home_node,
        rviz_node,
    ])
