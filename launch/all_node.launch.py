import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

def generate_launch_description():

    # Directories for each package
    antbot_cmd_vel_bridge_dir = get_package_share_directory('antbot_cmd_vel_bridge')
    antbot_ros2_control_dir = get_package_share_directory('antbot_ros2_control')

    return LaunchDescription([

        # antbot_cmd_vel_bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(antbot_cmd_vel_bridge_dir, 'launch', 'antbot_cmd_vel_bridge.launch.py'))
        ),

        # antbot_ros2_control 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(antbot_ros2_control_dir, 'launch', 'tricycle_drive.launch.py'))
        ),
    ])