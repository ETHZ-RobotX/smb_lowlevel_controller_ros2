from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
     
    robot_id = os.environ.get('ROBOT_ID', '000') # default to '000' if not set

    # Set RC channels based on robot ID
    if robot_id == '000':
        return LaunchDescription([
            LogInfo(msg="Environment variable ROBOT_ID is not set, exiting...")
        ])
    elif robot_id == '264':
        rc_channel_1 = 7
        rc_channel_2 = 8
    else:
        rc_channel_1 = 1
        rc_channel_2 = 2
    
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
                {"rc_channel_1": rc_channel_1},
                {"rc_channel_2": rc_channel_2},
                {"use_sim_time": False},
            ],
    )
    
    return LaunchDescription([
        low_level_controller,
    ])