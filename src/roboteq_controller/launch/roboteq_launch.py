from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'roboteq_controller',
            executable = 'speed_control_node',
            name = 'speed_controller',
        ),
        Node(
            package = 'speed_pub',
            executable = 'speed_pub_node',
            name = 'speed_publisher'
        )
    ])