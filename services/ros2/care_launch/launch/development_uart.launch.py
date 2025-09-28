#!/usr/bin/env python3

"""
C.A.R.E. Development UART Launch Configuration
Запускает режим разработки с прямым подключением к LD2450 или контроллеру через UART
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    uart_port_arg = DeclareLaunchArgument(
        'uart_port',
        default_value='/dev/ttyUSB0',
        description='UART port for radar/controller connection'
    )
    
    uart_mode_arg = DeclareLaunchArgument(
        'uart_mode',
        default_value='sensor',
        description='UART mode: sensor (direct LD2450) or controller (ESP32/STM32)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='256000',
        description='UART baud rate'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    use_safety_arg = DeclareLaunchArgument(
        'use_safety',
        default_value='true',
        description='Launch safety controller'
    )
    
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='300.0',
        description='Safety distance in mm'
    )
    
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='1',
        description='Device ID for UART connection'
    )
    
    # Package directories
    care_demo_share = FindPackageShare(package='care_demo_node').find('care_demo_node')
    rviz_config = PathJoinSubstitution([care_demo_share, 'config', 'care_demo.rviz'])
    
    # Demo Node в UART режиме
    uart_interface_node = Node(
        package='care_demo_node',
        executable='care_demo_node',
        name='care_uart_interface',
        output='screen',
        parameters=[{
            'data_source': 'uart',
            'uart_port': LaunchConfiguration('uart_port'),
            'uart_mode': LaunchConfiguration('uart_mode'),  # sensor или controller
            'baud_rate': LaunchConfiguration('baud_rate'),
            'device_id': LaunchConfiguration('device_id'),
            'update_rate': 10.0,
            'frame_id': 'radar_link',
            'base_frame': 'base_link',
            'publish_tf': True,
            'debug_mode': True,  # Включаем отладочную информацию
            'use_sim_time': False
        }],
        remappings=[
            ('/care/radar_targets', '/care/device_1/targets'),
            ('/care/device_status', '/care/device_1/status'),
            ('/care/uart_status', '/care/uart/status')
        ]
    )
    
    # Safety Controller Node
    safety_controller_node = Node(
        package='care_safety_controller_node',
        executable='care_safety_controller_node',
        name='care_safety_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_safety')),
        parameters=[{
            'max_devices': 1,  # В режиме разработки обычно одно устройство
            'safety_check_rate': 20.0,
            'default_safety_distance': LaunchConfiguration('safety_distance'),
            'default_safety_angle': 60.0,
            'emergency_hysteresis': 50.0,
            'min_trigger_time': 0.1,
            'log_events': True,
            'log_file': '/tmp/care_safety_development.log'
        }]
    )
    
    # Configuration Service Node (для настройки контроллера)
    config_service_node = Node(
        package='care_config_service',  # TODO: создать этот пакет
        executable='care_config_service',
        name='care_config_service',
        output='screen',
        parameters=[{
            'uart_port': LaunchConfiguration('uart_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'device_id': LaunchConfiguration('device_id'),
            'config_timeout': 5.0
        }]
    )
    
    # Static Transform Publishers
    static_transforms = [
        # Map to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        # Base_link to radar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar_link',
            arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'radar_link']
        )
    ]
    
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
    
    # UART Monitor Node (для отладки UART соединения)
    uart_monitor_node = Node(
        package='care_uart_monitor',  # TODO: создать этот пакет
        executable='care_uart_monitor',
        name='care_uart_monitor',
        output='screen',
        parameters=[{
            'uart_port': LaunchConfiguration('uart_port'),
            'monitor_rate': 1.0,
            'log_raw_data': True,
            'log_file': '/tmp/care_uart_monitor.log'
        }]
    )
    
    # Log startup message
    startup_log = LogInfo(
        msg="📡 C.A.R.E. Development Mode Started - UART connection active"
    )
    
    return LaunchDescription([
        # Arguments
        uart_port_arg,
        uart_mode_arg,
        baud_rate_arg,
        use_rviz_arg,
        use_safety_arg,
        safety_distance_arg,
        device_id_arg,
        
        # Startup message
        startup_log,
        
        # Core nodes
        uart_interface_node,
        safety_controller_node,
        config_service_node,
        uart_monitor_node,
        rviz_node
    ] + static_transforms)
