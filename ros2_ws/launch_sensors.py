"""
传感器驱动启动文件 — 单文件版（用于直接运行和测试）
与 mapless_nav/launch/sensors.launch.py 保持参数一致。

用法:
  ros2 launch launch_sensors.py time_sync_mode:=ptp   # PTP 时间同步
  ros2 launch launch_sensors.py time_sync_mode:=system # 系统时钟

输出:
  /unilidar/cloud    — Unitree L2 点云
  /unilidar/imu      — Unitree L2 IMU
  /berxel/*          — Berxel P100R RGB-D 图像
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------
    launch_l2 = LaunchConfiguration('launch_l2')
    launch_berxel = LaunchConfiguration('launch_berxel')
    time_sync_mode = LaunchConfiguration('time_sync_mode')

    declare_launch_l2 = DeclareLaunchArgument(
        'launch_l2', default_value='true',
        description='Launch Unitree L2 driver')
    declare_launch_berxel = DeclareLaunchArgument(
        'launch_berxel', default_value='true',
        description='Launch Berxel P100R driver')
    declare_time_sync_mode = DeclareLaunchArgument(
        'time_sync_mode', default_value='ptp',
        choices=['ptp', 'system', 'none'],
        description='Time sync mode: ptp (PTP/gPTP hardware), system (ROS clock), none (driver default)')

    # PTP 模式时使用 LiDAR 硬件时间戳
    l2_use_system_ts = PythonExpression([
        '"false" if "', time_sync_mode, '" == "ptp" else "true"'
    ])

    # --------------------------------------------------------------------------
    # Unitree L2 LiDAR
    # --------------------------------------------------------------------------
    unitree_lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[{
            'initialize_type': 2,
            'work_mode': 1,
            'use_system_timestamp': l2_use_system_ts,
            'range_min': 0.0,
            'range_max': 100.0,
            'cloud_scan_num': 18,
            'lidar_port': 6101,
            'lidar_ip': '192.168.1.62',   # Unitree L2 LiDAR
            'local_port': 6201,
            'local_ip': '192.168.1.2',    # NUC secondary IP (L2 sends to this addr)
            'cloud_frame': 'unilidar_lidar',
            'cloud_topic': 'unilidar/cloud',
            'imu_frame': 'unilidar_imu',
            'imu_topic': 'unilidar/imu',
        }],
        condition=IfCondition(launch_l2),
    )

    # --------------------------------------------------------------------------
    # Berxel P100R RGB-D Camera
    # --------------------------------------------------------------------------
    berxel_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('berxel_camera_ros2'),
                'launch',
                'berxel_camera_iHawk100.py'
            ])
        ]),
        condition=IfCondition(launch_berxel),
    )

    # --------------------------------------------------------------------------
    # RViz
    # --------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('berxel_camera_ros2'), 'rviz',
            'berxel_default.rviz'])],
        condition=IfCondition(launch_berxel),
    )

    return LaunchDescription([
        declare_launch_l2,
        declare_launch_berxel,
        declare_time_sync_mode,
        unitree_lidar_node,
        berxel_camera_launch,
        rviz_node,
    ])
