#!/usr/bin/env python3
"""
Launch file for sensor drivers only

This launches:
1. Unitree L2 LiDAR driver
2. Livox Mid-70 LiDAR driver
3. Berxel P100R RGB-D camera driver

Time synchronization modes:
  - ptp:   Use PTP/gPTP hardware timestamping (requires linuxptp or similar
           running on the host, and PTP enabled in LiDAR firmware).
           L2: use_system_timestamp=false, Livox: auto-detects PTP from packet header.
  - system: Use ROS system clock timestamp (no external sync).
  - none:  Use driver default (same as 'system' for most drivers).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------
    launch_l2 = LaunchConfiguration('launch_l2')
    launch_mid70 = LaunchConfiguration('launch_mid70')
    launch_berxel = LaunchConfiguration('launch_berxel')
    time_sync_mode = LaunchConfiguration('time_sync_mode')

    declare_launch_l2 = DeclareLaunchArgument(
        'launch_l2', default_value='true',
        description='Launch Unitree L2 driver'
    )
    declare_launch_mid70 = DeclareLaunchArgument(
        'launch_mid70', default_value='true',
        description='Launch Livox Mid-70 driver'
    )
    declare_launch_berxel = DeclareLaunchArgument(
        'launch_berxel', default_value='true',
        description='Launch Berxel P100R driver'
    )
    declare_time_sync_mode = DeclareLaunchArgument(
        'time_sync_mode', default_value='ptp',
        choices=['ptp', 'system', 'none'],
        description='Time sync mode: ptp (PTP/gPTP hardware), system (ROS clock), none (driver default)'
    )

    # When time_sync_mode == 'ptp', use LiDAR hardware (PTP-synced) timestamps
    l2_use_system_ts = PythonExpression([
        '"false" if "', time_sync_mode, '" == "ptp" else "true"'
    ])

    # ==========================================================================
    # Unitree L2 LiDAR
    # ==========================================================================
    l2_driver_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[{
            'initialize_type': 2,
            'work_mode': 0,
            'use_system_timestamp': l2_use_system_ts,
            'range_min': 0.0,
            'range_max': 100.0,
            'cloud_scan_num': 18,
            'lidar_port': 6101,
            'lidar_ip': '192.168.1.62',   # Unitree L2 LiDAR
            'local_port': 6201,
            'local_ip': '192.168.1.1',    # NUC IP that currently receives L2 UDP
            'cloud_frame': 'unilidar_lidar',
            'cloud_topic': 'unilidar/cloud',
            'imu_frame': 'unilidar_imu',
            'imu_topic': 'unilidar/imu',
        }],
        condition=IfCondition(launch_l2),
    )

    # ==========================================================================
    # Livox Mid-70 LiDAR
    # ==========================================================================
    # Note: Livox driver auto-detects PTP from LiDAR packet headers (time_type field).
    # PTP must be enabled on the LiDAR itself (via Livox Viewer or firmware config).
    # No ROS-level toggle is needed — the driver uses hardware timestamps when present.
    mid70_config_path = '/path/to/livox_lidar_config.json'  # Update path

    mid70_driver_node = Node(
        package='livox_ros2_driver',
        executable='livox_ros2_driver_node',
        name='livox_mid70_driver',
        output='screen',
        parameters=[{
            'xfer_format': 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '',
            'user_config_path': mid70_config_path,
        }],
        condition=IfCondition(launch_mid70),
    )

    # ==========================================================================
    # Berxel P100R RGB-D Camera
    # ==========================================================================
    berxel_driver_node = Node(
        package='mapless_nav',
        executable='berxel_camera_node.py',
        name='berxel_camera_driver',
        output='screen',
        parameters=[{
            'color_topic': '/berxel/color/image_raw',
            'depth_topic': '/berxel/depth/image_raw',
            'camera_info_topic': '/berxel/depth/camera_info',
            'frame_id': 'berxel_link',
            'depth_frame_id': 'berxel_depth_optical_frame',
            'fps': 30,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
        }],
        condition=IfCondition(launch_berxel),
    )

    return LaunchDescription([
        # Arguments
        declare_launch_l2,
        declare_launch_mid70,
        declare_launch_berxel,
        declare_time_sync_mode,

        # Drivers
        l2_driver_node,
        # Note: Uncomment the remaining drivers after their local launch
        # configuration has been validated.
        # mid70_driver_node,
        # berxel_driver_node,
    ])
