#!/usr/bin/env python3
"""
Launch file for the complete mapless navigation system

This launches:
1. Sensor drivers (Unitree L2, Mid-70, Berxel P100R)
2. Point cloud fusion
3. YOLO tracker
4. Following controller
5. Nav2 stack (optional)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('mapless_nav')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav2 = LaunchConfiguration('use_nav2')
    enable_following = LaunchConfiguration('enable_following')
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_patrol = LaunchConfiguration('enable_patrol')
    odom_mode = LaunchConfiguration('odom_mode')

    # Config files
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_mapless_params.yaml')
    fusion_params_file = os.path.join(pkg_share, 'config', 'fusion_params.yaml')
    tracker_params_file = os.path.join(pkg_share, 'config', 'tracker_params.yaml')
    scan_conversion_params_file = os.path.join(pkg_share, 'config', 'scan_conversion_params.yaml')
    return_home_params_file = os.path.join(pkg_share, 'config', 'return_home_params.yaml')
    patrol_params_file = os.path.join(pkg_share, 'config', 'patrol_params.yaml')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='false',
        description='Launch Nav2 stack'
    )
    
    declare_enable_following = DeclareLaunchArgument(
        'enable_following',
        default_value='true',
        description='Enable following controller'
    )
    
    declare_enable_visualization = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization outputs'
    )

    declare_enable_patrol = DeclareLaunchArgument(
        'enable_patrol',
        default_value='false',
        description='Enable waypoint patrol node'
    )

    declare_odom_mode = DeclareLaunchArgument(
        'odom_mode',
        default_value='fake',
        choices=['fake', 'l2_imu', 'external'],
        description='Odometry mode: fake (testing), l2_imu (L2 IMU), external (FAST_LIO/slam_toolbox)'
    )
    
    # ==========================================================================
    # Point Cloud Fusion Node
    # ==========================================================================
    pointcloud_fusion_node = Node(
        package='mapless_nav',
        executable='pointcloud_fusion_node',
        name='pointcloud_fusion_node',
        output='screen',
        parameters=[fusion_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # ==========================================================================
    # Depth to PointCloud Node
    # ==========================================================================
    depth_to_pointcloud_node = Node(
        package='mapless_nav',
        executable='depth_to_pointcloud_node',
        name='depth_to_pointcloud_node',
        output='screen',
        parameters=[fusion_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # ==========================================================================
    # Target Tracker Node (YOLO + ByteTrack)
    # ==========================================================================
    target_tracker_node = Node(
        package='mapless_nav',
        executable='target_tracker_node.py',
        name='target_tracker_node',
        output='screen',
        parameters=[tracker_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # ==========================================================================
    # Following Controller Node
    # ==========================================================================
    following_controller_node = Node(
        package='mapless_nav',
        executable='following_controller_node.py',
        name='following_controller_node',
        output='screen',
        parameters=[tracker_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_following)
    )
    
    # ==========================================================================
    # PointCloud2 to LaserScan Conversion Node
    # ==========================================================================
    pointcloud_to_scan_node = Node(
        package='mapless_nav',
        executable='pointcloud_to_scan_node.py',
        name='pointcloud_to_scan_node',
        output='screen',
        parameters=[scan_conversion_params_file, {'use_sim_time': use_sim_time}]
    )

    # ==========================================================================
    # Return Home Node
    # ==========================================================================
    return_home_node = Node(
        package='mapless_nav',
        executable='return_home_node.py',
        name='return_home_node',
        output='screen',
        parameters=[return_home_params_file, {'use_sim_time': use_sim_time}]
    )

    # ==========================================================================
    # Waypoint Patrol Node (optional)
    # ==========================================================================
    waypoint_patrol_node = Node(
        package='mapless_nav',
        executable='waypoint_patrol_node.py',
        name='waypoint_patrol_node',
        output='screen',
        parameters=[patrol_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_patrol)
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
    # Static TF Publishers for sensor frames
    # ==========================================================================
    
    # Base to Mid-70 (forward, slightly elevated)
    tf_base_to_mid70 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_mid70',
        arguments=['0.15', '0', '0.3', '0', '0', '0', 'base_link', 'livox_frame']
    )
    
    # Base to Unitree L2 (top of robot)
    tf_base_to_l2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_l2',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'unilidar_lidar']
    )
    
    # Base to Berxel camera (front, angled down slightly)
    tf_base_to_berxel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_berxel',
        arguments=['0.12', '0', '0.25', '0', '0.1', '0', 'base_link', 'berxel_link']
    )
    
    # Berxel link to depth optical frame
    tf_berxel_to_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_berxel_to_depth',
        # Standard camera optical frame rotation: Z forward, X right, Y down
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'berxel_link', 'berxel_depth_optical_frame']
    )
    
    # ==========================================================================
    # Nav2 Stack (optional) - Only include if nav2_bringup is installed
    # ==========================================================================
    nav2_actions = []
    try:
        from ament_index_python.packages import get_package_share_directory as get_pkg
        nav2_bringup_dir = get_pkg('nav2_bringup')
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'autostart': 'true'
            }.items(),
            condition=IfCondition(use_nav2)
        )
        nav2_actions.append(nav2_launch)
    except Exception:
        print("Nav2 bringup not installed, skipping Nav2 launch")
    
    # ==========================================================================
    # RViz (optional)
    # ==========================================================================
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mapless_nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(enable_visualization)
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_use_nav2,
        declare_enable_following,
        declare_enable_visualization,
        declare_enable_patrol,
        declare_odom_mode,

        # Odometry
        real_odom_node,

        # TF
        tf_base_to_mid70,
        tf_base_to_l2,
        tf_base_to_berxel,
        tf_berxel_to_depth,

        # Processing
        depth_to_pointcloud_node,
        pointcloud_fusion_node,
        pointcloud_to_scan_node,

        # Perception + Control
        target_tracker_node,
        following_controller_node,

        # Navigation services
        return_home_node,
        waypoint_patrol_node,

        # Nav2 (optional)
        *nav2_actions,

        # RViz (optional)
        rviz_node,
    ])
