#!/usr/bin/env python3

"""
C.A.R.E. Production CAN Launch Configuration
Запускает продакшн режим с реальными CAN контроллерами
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate',
        default_value='500000',
        description='CAN bitrate'
    )
    
    max_devices_arg = DeclareLaunchArgument(
        'max_devices',
        default_value='15',
        description='Maximum number of CAN devices'
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
    
    use_monitoring_arg = DeclareLaunchArgument(
        'use_monitoring',
        default_value='true',
        description='Enable system monitoring and logging'
    )
    
    # CAN Interface Node - читает данные с CAN шины
    can_interface_node = Node(
        package='care_can_interface_node',
        executable='care_can_interface_node',
        name='care_can_interface',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'can_bitrate': LaunchConfiguration('can_bitrate'),
            'max_devices': LaunchConfiguration('max_devices'),
            'target_timeout_ms': 500,
            'status_timeout_ms': 2000
        }]
    )
    
    # Safety Controller Node
    safety_controller_node = Node(
        package='care_safety_controller_node',
        executable='care_safety_controller_node',
        name='care_safety_controller',
        output='screen',
        parameters=[{
            'max_devices': LaunchConfiguration('max_devices'),
            'safety_check_rate': 20.0,
            'default_safety_distance': LaunchConfiguration('safety_distance'),
            'default_safety_angle': LaunchConfiguration('safety_angle'),
            'emergency_hysteresis': 50.0,
            'min_trigger_time': 0.1,
            'log_events': True,
            'log_file': '/var/log/care_safety_production.log'
        }]
    )
    
    # System Monitor Node (опционально)
    system_monitor_node = Node(
        package='care_system_monitor',  # TODO: создать этот пакет
        executable='care_system_monitor',
        name='care_system_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_monitoring')),
        parameters=[{
            'monitor_rate': 1.0,
            'log_file': '/var/log/care_system_monitor.log',
            'alert_thresholds': {
                'device_offline_time': 5.0,
                'emergency_duration': 30.0,
                'message_rate_min': 5.0
            }
        }]
    )
    
    # Static Transform Publishers для продакшн системы
    static_transforms = GroupAction([
        # Map to base_link (может быть настроено под конкретную установку)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        
        # Base_link to radar frames для каждого устройства
        # Device 1
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar_link_1',
            arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'radar_link_1']
        ),
        
        # Device 2 (пример для множественных радаров)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar_link_2',
            arguments=['0.2', '0.5', '0.1', '0', '0', '0.5236', 'base_link', 'radar_link_2']  # 30° поворот
        )
    ])
    
    # Log startup message
    startup_log = LogInfo(
        msg="🚌 C.A.R.E. Production Mode Started - Real CAN controllers active"
    )
    
    return LaunchDescription([
        # Arguments
        can_interface_arg,
        can_bitrate_arg,
        max_devices_arg,
        safety_distance_arg,
        safety_angle_arg,
        use_monitoring_arg,
        
        # Startup message
        startup_log,
        
        # Core nodes
        can_interface_node,
        safety_controller_node,
        system_monitor_node,
        static_transforms
    ])
