from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            namespace='ps5_controlloer',
            executable='joy_node'
        ),
        Node(
            package='motor_control',
            executable='listener'
        )
    ])
