#!/usr/bin/env python3
"""
Odometry Launch File
Launches the odometry backend (real_odom_node with various modes).

Usage:
  # IMU-based dead reckoning (default, uses L2 IMU)
  ros2 launch mapless_nav odometry.launch.py

  # External odometry (e.g. FAST_LIO or slam_toolbox providing /odom_raw)
  ros2 launch mapless_nav odometry.launch.py mode:=external

  # Fake odometry for testing
  ros2 launch mapless_nav odometry.launch.py mode:=fake
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    publish_rate = LaunchConfiguration('publish_rate')

    declare_mode = DeclareLaunchArgument(
        'mode', default_value='l2_imu',
        choices=['l2_imu', 'external', 'fake'],
        description='Odometry backend: l2_imu (L2 IMU dead reckoning), '
                    'external (relay from /odom_raw), fake (static)'
    )
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate', default_value='50.0',
        description='Odometry publish rate in Hz'
    )

    real_odom_node = Node(
        package='mapless_nav',
        executable='real_odom_node.py',
        name='real_odom_node',
        output='screen',
        parameters=[{
            'mode': mode,
            'publish_rate': publish_rate,
        }]
    )

    return LaunchDescription([
        declare_mode,
        declare_publish_rate,
        real_odom_node,
    ])
