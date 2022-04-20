from launch import LaunchDescription
from launch_ros.actions import Node
import os




def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros'
            name='cartographer_node'
            executable='cartographer_node'
            parameters=[
                configuration_directory='/home/ubuntu/ros2_ws/src/cart_start/configuration_files',
                configuration_basename= 'wilie_2d.lua'
            ]
        )
    ])