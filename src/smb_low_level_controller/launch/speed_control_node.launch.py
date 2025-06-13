from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    # Low-level gazebo controller node
    low_level_controller = Node(
        package="smb_low_level_controller",
        executable="speed_control_node",
        name="speed_control_node",
        output="screen",
        parameters=[
                {"port": "/dev/ttyUSB0"},
                {"baudrate": 115200},
                {"motor_1_channel": 1},
                {"motor_2_channel": 2},
                {"send_cmd": 1},
                {"rc_channel_1": 1},
                {"rc_channel_2": 2},
            ],
    )
    
    return LaunchDescription([
        low_level_controller,
    ])