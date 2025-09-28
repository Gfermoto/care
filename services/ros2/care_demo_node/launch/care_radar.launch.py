from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='care_radar_publisher',
            executable='care_radar_node',
            name='care_radar_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'radar_frame': 'radar_link',
                'base_frame': 'base_link',
                'safety_distance': 1.5,
                'safety_angle': 60.0,
                'update_rate': 10.0
            }]
        )
    ])

