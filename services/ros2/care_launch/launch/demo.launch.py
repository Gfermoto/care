#!/usr/bin/env python3

"""
C.A.R.E. Demo Launch Configuration
Запускает демо режим с Mock данными, RViz2 визуализацией и системой безопасности
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
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
    
    safety_angle_arg = DeclareLaunchArgument(
        'safety_angle',
        default_value='60.0',
        description='Safety angle in degrees'
    )
    
    # Package directories
    care_launch_share = FindPackageShare(package='care_launch').find('care_launch')
    care_demo_share = FindPackageShare(package='care_demo_node').find('care_demo_node')
    
    # RViz config path
    rviz_config = PathJoinSubstitution([care_demo_share, 'config', 'care_demo.rviz'])
    
    # Demo Node - генерирует Mock данные
    demo_node = Node(
        package='care_demo_node',
        executable='care_demo_node',
        name='care_demo_node',
        output='screen',
        parameters=[{
            'data_source': 'mock',
            'mock_scenario': 'parking',
            'realistic_ld2450_format': True,
            'update_rate': 10.0,
            'safety_distance': LaunchConfiguration('safety_distance'),
            'safety_angle': LaunchConfiguration('safety_angle'),
            'frame_id': 'radar_link',
            'base_frame': 'base_link',
            'publish_tf': True,
            'use_sim_time': False
        }],
        remappings=[
            ('/care/radar_targets', '/care/demo/targets'),
            ('/care/safety_zone', '/care/demo/safety_zone'),
            ('/care/status', '/care/demo/status')
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
            'max_devices': 15,
            'safety_check_rate': 20.0,
            'default_safety_distance': LaunchConfiguration('safety_distance'),
            'default_safety_angle': LaunchConfiguration('safety_angle'),
            'emergency_hysteresis': 50.0,
            'min_trigger_time': 0.1,
            'log_events': True,
            'log_file': '/tmp/care_safety_demo.log'
        }]
    )
    
    # Static Transform Publishers
    static_transforms = GroupAction([
        # Map to base_link transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        # Base_link to radar_link transform  
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar_link',
            arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'radar_link']
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
    
    # Log startup message
    startup_log = LogInfo(
        msg="🎭 C.A.R.E. Demo Mode Started - Mock radar data with safety system"
    )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        use_safety_arg,
        safety_distance_arg,
        safety_angle_arg,
        
        # Startup message
        startup_log,
        
        # Core nodes
        demo_node,
        safety_controller_node,
        static_transforms,
        rviz_node
    ])
