#!/usr/bin/env python3

"""
C.A.R.E. Full System Launch Configuration
Запускает полную систему со всеми компонентами: CAN, Demo, Safety, Monitoring
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mixed',
        description='System mode: mixed (CAN+Demo), can_only, demo_only'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    max_devices_arg = DeclareLaunchArgument(
        'max_devices',
        default_value='15',
        description='Maximum number of CAN devices'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    use_monitoring_arg = DeclareLaunchArgument(
        'use_monitoring',
        default_value='true',
        description='Enable comprehensive system monitoring'
    )
    
    use_web_dashboard_arg = DeclareLaunchArgument(
        'use_web_dashboard',
        default_value='false',
        description='Launch web dashboard (requires Node.js services)'
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
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level: DEBUG, INFO, WARN, ERROR'
    )
    
    # Package directories
    care_launch_share = FindPackageShare(package='care_launch').find('care_launch')
    care_demo_share = FindPackageShare(package='care_demo_node').find('care_demo_node')
    rviz_config = PathJoinSubstitution([care_demo_share, 'config', 'care_demo.rviz'])
    
    # CAN Interface Node (для режимов can_only и mixed)
    can_interface_node = Node(
        package='care_can_interface_node',
        executable='care_can_interface_node',
        name='care_can_interface',
        output='screen',
        condition=IfCondition(
            # Запускается если mode = can_only ИЛИ mode = mixed
            LaunchConfiguration('mode').matches('can_only').or_(
                LaunchConfiguration('mode').matches('mixed')
            )
        ),
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'can_bitrate': 500000,
            'max_devices': LaunchConfiguration('max_devices'),
            'target_timeout_ms': 500,
            'status_timeout_ms': 2000
        }]
    )
    
    # Demo Node (для режимов demo_only и mixed)
    demo_node = Node(
        package='care_demo_node',
        executable='care_demo_node',
        name='care_demo_node',
        output='screen',
        condition=IfCondition(
            # Запускается если mode = demo_only ИЛИ mode = mixed
            LaunchConfiguration('mode').matches('demo_only').or_(
                LaunchConfiguration('mode').matches('mixed')
            )
        ),
        parameters=[{
            'data_source': 'mock',
            'mock_scenario': 'mixed_traffic',  # Более сложный сценарий для полной системы
            'realistic_ld2450_format': True,
            'update_rate': 10.0,
            'safety_distance': LaunchConfiguration('safety_distance'),
            'safety_angle': LaunchConfiguration('safety_angle'),
            'frame_id': 'radar_link_demo',
            'base_frame': 'base_link',
            'publish_tf': True,
            'device_id': 99,  # Специальный ID для демо
            'use_sim_time': False
        }],
        remappings=[
            ('/care/radar_targets', '/care/demo/targets'),
            ('/care/safety_zone', '/care/demo/safety_zone'),
            ('/care/status', '/care/demo/status')
        ]
    )
    
    # Safety Controller Node (всегда запускается)
    safety_controller_node = Node(
        package='care_safety_controller_node',
        executable='care_safety_controller_node',
        name='care_safety_controller',
        output='screen',
        parameters=[{
            'max_devices': LaunchConfiguration('max_devices'),
            'safety_check_rate': 50.0,  # Более высокая частота для полной системы
            'default_safety_distance': LaunchConfiguration('safety_distance'),
            'default_safety_angle': LaunchConfiguration('safety_angle'),
            'emergency_hysteresis': 50.0,
            'min_trigger_time': 0.05,  # Более быстрая реакция
            'log_events': True,
            'log_file': '/var/log/care_safety_full_system.log'
        }]
    )
    
    # System Monitor Node
    system_monitor_node = Node(
        package='care_system_monitor',  # TODO: создать
        executable='care_system_monitor',
        name='care_system_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_monitoring')),
        parameters=[{
            'monitor_rate': 2.0,  # Более частый мониторинг
            'log_file': '/var/log/care_system_monitor_full.log',
            'alert_thresholds': {
                'device_offline_time': 3.0,
                'emergency_duration': 15.0,
                'message_rate_min': 5.0,
                'cpu_usage_max': 0.8,
                'memory_usage_max': 0.9
            },
            'enable_alerts': True,
            'alert_email': 'admin@care-system.com'  # TODO: настроить
        }]
    )
    
    # Performance Monitor Node
    performance_monitor_node = Node(
        package='care_performance_monitor',  # TODO: создать
        executable='care_performance_monitor',
        name='care_performance_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_monitoring')),
        parameters=[{
            'monitor_rate': 1.0,
            'log_performance_data': True,
            'performance_log_file': '/var/log/care_performance.log',
            'track_message_rates': True,
            'track_latencies': True,
            'track_resource_usage': True
        }]
    )
    
    # Data Logger Node (для записи всех данных)
    data_logger_node = Node(
        package='care_data_logger',  # TODO: создать
        executable='care_data_logger',
        name='care_data_logger',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_monitoring')),
        parameters=[{
            'log_targets': True,
            'log_safety_events': True,
            'log_system_status': True,
            'log_format': 'rosbag2',  # или 'csv', 'json'
            'log_directory': '/var/log/care_data/',
            'max_log_size_mb': 1000,
            'auto_rotate': True
        }]
    )
    
    # Web Dashboard Bridge (если включен)
    web_dashboard_bridge = Node(
        package='care_web_bridge',  # TODO: создать или использовать Node.js
        executable='care_web_bridge',
        name='care_web_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_web_dashboard')),
        parameters=[{
            'web_port': 8080,
            'websocket_port': 8081,
            'enable_rest_api': True,
            'enable_websocket': True,
            'cors_enabled': True
        }]
    )
    
    # Static Transform Publishers для полной системы
    static_transforms = GroupAction([
        # Map to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        
        # Multiple radar positions для CAN устройств
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar_link_1',
            arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'radar_link_1']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar_link_2',
            arguments=['0.2', '0.3', '0.1', '0', '0', '0.5236', 'base_link', 'radar_link_2']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar_link_3',
            arguments=['0.2', '-0.3', '0.1', '0', '0', '-0.5236', 'base_link', 'radar_link_3']
        ),
        
        # Demo radar position
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar_link_demo',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'radar_link_demo']
        )
    ])
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # Startup and status messages
    startup_log = LogInfo(
        msg="🚀 C.A.R.E. Full System Started - All components active"
    )
    
    status_log = LogInfo(
        msg="📊 System Configuration: Mode={}, CAN={}, Devices={}, Safety={}mm/{}°".format(
            LaunchConfiguration('mode'),
            LaunchConfiguration('can_interface'),
            LaunchConfiguration('max_devices'),
            LaunchConfiguration('safety_distance'),
            LaunchConfiguration('safety_angle')
        )
    )
    
    return LaunchDescription([
        # Arguments
        mode_arg,
        can_interface_arg,
        max_devices_arg,
        use_rviz_arg,
        use_monitoring_arg,
        use_web_dashboard_arg,
        safety_distance_arg,
        safety_angle_arg,
        log_level_arg,
        
        # Startup messages
        startup_log,
        status_log,
        
        # Core system nodes
        can_interface_node,
        demo_node,
        safety_controller_node,
        
        # Monitoring and logging
        system_monitor_node,
        performance_monitor_node,
        data_logger_node,
        
        # Web interface
        web_dashboard_bridge,
        
        # Visualization and transforms
        static_transforms,
        rviz_node
    ])
