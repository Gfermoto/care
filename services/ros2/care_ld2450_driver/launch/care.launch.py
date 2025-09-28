#!/usr/bin/env python3

"""
C.A.R.E. Radar Launch File
Launches the C.A.R.E. radar system with LD2450 driver and safety controller
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare(package='care_ld2450_driver').find('care_ld2450_driver')
    config_dir = PathJoinSubstitution([pkg_share, 'config'])
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    radar_port_arg = DeclareLaunchArgument(
        'radar_port',
        default_value='/dev/ttyUSB0',
        description='Radar serial port'
    )
    
    radar_baud_arg = DeclareLaunchArgument(
        'radar_baud',
        default_value='256000',
        description='Radar baud rate'
    )
    
    can_enabled_arg = DeclareLaunchArgument(
        'can_enabled',
        default_value='true',
        description='Enable CAN interface'
    )
    
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='300.0',
        description='Safety distance in mm'
    )
    
    safety_angle_arg = DeclareLaunchArgument(
        'safety_angle',
        default_value='60.0',
        description='Safety angle in degrees'
    )
    
    # LD2450 Driver Node
    ld2450_driver_node = Node(
        package='care_ld2450_driver',
        executable='ld2450_driver_node',
        name='ld2450_driver',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'radar_port': LaunchConfiguration('radar_port'),
            'radar_baud': LaunchConfiguration('radar_baud'),
            'can_enabled': LaunchConfiguration('can_enabled'),
            'safety_distance': LaunchConfiguration('safety_distance'),
            'safety_angle': LaunchConfiguration('safety_angle'),
            'frame_id': 'radar_link',
            'publish_rate': 20.0,
            'debug': False
        }],
        remappings=[
            ('/radar/targets', '/care/radar/targets'),
            ('/radar/safety_status', '/care/radar/safety_status'),
            ('/radar/diagnostics', '/care/radar/diagnostics')
        ]
    )
    
    # Safety Controller Node
    safety_controller_node = Node(
        package='care_ld2450_driver',
        executable='safety_controller_node',
        name='safety_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'safety_distance': LaunchConfiguration('safety_distance'),
            'safety_angle': LaunchConfiguration('safety_angle'),
            'emergency_stop_timeout': 1.0,
            'can_enabled': LaunchConfiguration('can_enabled')
        }],
        remappings=[
            ('/radar/targets', '/care/radar/targets'),
            ('/radar/safety_status', '/care/radar/safety_status'),
            ('/safety/emergency_stop', '/care/safety/emergency_stop')
        ]
    )
    
    # CAN Bridge Node (if enabled)
    can_bridge_node = Node(
        package='care_can_bridge',
        executable='can_bridge_node',
        name='can_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'can_interface': 'can0',
            'can_bitrate': 500000,
            'emergency_stop_id': 0x100,
            'target_data_id': 0x200
        }],
        condition=LaunchConfiguration('can_enabled'),
        remappings=[
            ('/safety/emergency_stop', '/care/safety/emergency_stop'),
            ('/radar/targets', '/care/radar/targets')
        ]
    )
    
    # Static Transform Publisher
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='radar_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'radar_link'],
        output='screen'
    )
    
    # Log info
    log_info = LogInfo(
        msg="C.A.R.E. Radar System Starting..."
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        radar_port_arg,
        radar_baud_arg,
        can_enabled_arg,
        safety_distance_arg,
        safety_angle_arg,
        log_info,
        ld2450_driver_node,
        safety_controller_node,
        can_bridge_node,
        static_tf_publisher
    ])
