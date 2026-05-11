#!/usr/bin/env python3
"""
Patrol Navigation Launch — COD Architecture: Multi-Waypoint SLAM Mode

Mirrors cod_bringup/multiplenav_launch.py pattern:
  1. Sensor processing (pointcloud fusion + self-filter + scan conversion)
  2. Odometry (L2 IMU / external / fake)
  3. Nav2 bringup (all lifecycle nodes)
  4. Return-home service
  5. Waypoint patrol (optional, started via /patrol/start service)

Usage:
  ros2 launch mapless_nav_v2 patrol_nav.launch.py
  ros2 launch mapless_nav_v2 patrol_nav.launch.py odom_mode:=l2_imu
  ros2 launch mapless_nav_v2 patrol_nav.launch.py enable_patrol:=true
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

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_mode = LaunchConfiguration('odom_mode')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_patrol = LaunchConfiguration('enable_patrol')

    # Config files
    nav2_params = os.path.join(v2_share, 'config', 'nav2_params.yaml')
    fusion_params = os.path.join(v1_share, 'config', 'fusion_params.yaml')
    scan_params = os.path.join(v1_share, 'config', 'scan_conversion_params.yaml')
    return_home_params = os.path.join(v1_share, 'config', 'return_home_params.yaml')
    patrol_params = os.path.join(v1_share, 'config', 'patrol_params.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_odom_mode = DeclareLaunchArgument(
        'odom_mode', default_value='fake',
        choices=['fake', 'l2_imu', 'external'],
        description='Odometry mode')
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz', default_value='true')
    declare_enable_patrol = DeclareLaunchArgument(
        'enable_patrol', default_value='false',
        description='Enable waypoint patrol at startup')

    # =========================================================================
    # Odometry
    # =========================================================================
    odom_node = Node(
        package='mapless_nav',
        executable='real_odom_node.py',
        name='real_odom_node',
        output='screen',
        parameters=[{'mode': odom_mode, 'use_sim_time': use_sim_time}])

    # =========================================================================
    # Static TF: base_link -> unilidar_lidar (L2 LiDAR mount)
    # =========================================================================
    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_lidar',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'unilidar_lidar'])

    # =========================================================================
    # Sensor Processing Pipeline (same as COD's cpp_lidar_filter + pointcloud_to_laserscan)
    # =========================================================================
    fusion_node = Node(
        package='mapless_nav',
        executable='pointcloud_fusion_node',
        name='pointcloud_fusion_node',
        output='screen',
        parameters=[fusion_params, {'use_sim_time': use_sim_time}])

    scan_node = Node(
        package='mapless_nav',
        executable='pointcloud_to_scan_node.py',
        name='pointcloud_to_scan_node',
        output='screen',
        parameters=[scan_params, {'use_sim_time': use_sim_time}])

    # =========================================================================
    # Nav2 Bringup (COD-style navigation_launch)
    # =========================================================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(v2_share, 'launch', 'bringup.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items())

    # =========================================================================
    # Return Home Node (always on)
    # =========================================================================
    return_home_node = Node(
        package='mapless_nav',
        executable='return_home_node.py',
        name='return_home_node',
        output='screen',
        parameters=[return_home_params, {'use_sim_time': use_sim_time}])

    # =========================================================================
    # Waypoint Patrol Node (optional)
    # =========================================================================
    patrol_node = Node(
        package='mapless_nav',
        executable='waypoint_patrol_node.py',
        name='waypoint_patrol_node',
        output='screen',
        parameters=[patrol_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_patrol))

    # =========================================================================
    # RViz
    # =========================================================================
    rviz_config = os.path.join(v2_share, 'rviz', 'patrol_nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(enable_rviz))

    # Delay Nav2 to let TF + odometry stabilize
    delayed_nav2 = TimerAction(period=2.0, actions=[nav2_bringup])

    return LaunchDescription([
        declare_use_sim_time,
        declare_odom_mode,
        declare_enable_rviz,
        declare_enable_patrol,

        # Odometry + TF (must be first)
        odom_node,
        tf_base_to_lidar,

        # Sensor processing
        fusion_node,
        scan_node,

        # Nav2 (delayed)
        delayed_nav2,

        # Higher-level services
        return_home_node,
        patrol_node,

        # Visualization
        rviz_node,
    ])
