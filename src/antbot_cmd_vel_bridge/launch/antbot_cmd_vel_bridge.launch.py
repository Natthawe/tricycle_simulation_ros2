from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_cfg = PathJoinSubstitution([
        FindPackageShare('antbot_cmd_vel_bridge'),
        'config',
        'bridge.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_cfg,
            description='YAML file with parameters for cmd_vel_bridge'),

        Node(
            package='antbot_cmd_vel_bridge',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        )
    ])
