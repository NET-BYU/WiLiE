from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    joystick_node = Node(
        package="joy",
        executable="joy_node",
    )
    
    motor_ctrl_node = Node(
        package="motor_control",
        executable="listener"
    )

    ld.add_action(joystick_node)
    ld.add_action(motor_ctrl_node)
    
    return ld
