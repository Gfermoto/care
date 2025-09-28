from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='care_demo_node',
            executable='care_demo_node',
            name='care_demo_node',
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

