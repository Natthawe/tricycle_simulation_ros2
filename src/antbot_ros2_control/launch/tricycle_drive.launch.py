from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ===== Launch Arguments (pose + sim time) =====
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    roll   = LaunchConfiguration('roll')
    pitch  = LaunchConfiguration('pitch')
    yaw    = LaunchConfiguration('yaw')

    # ===== Paths in package =====
    pkg_description_share   = FindPackageShare('antbot_description')
    pkg_share   = FindPackageShare('antbot_ros2_control')
    urdf_file   = PathJoinSubstitution([pkg_description_share, 'urdf', 'robots', 'tricycle_drive.xacro.urdf'])
    world_file  = PathJoinSubstitution([pkg_share, 'worlds', 'warehouse.sdf'])
    rviz_file   = PathJoinSubstitution([pkg_description_share, 'rviz', 'tricycle.rviz'])
    gz_bridge_params_path = os.path.join(
        get_package_share_directory('antbot_ros2_control'),
        'config',
        'gz_bridge.yaml'
    )
    ekf_params_path = get_package_share_directory('antbot_ros2_control')

    # ===== robot_description from xacro =====
    robot_description_content = Command([FindExecutable(name='xacro'), ' ', urdf_file])
    robot_description = {'robot_description': robot_description_content,
                         'use_sim_time': use_sim_time}

    # ===== controller config =====
    robot_controllers = PathJoinSubstitution([pkg_share, 'config', 'tricycle_drive_controller.yaml'])

    # ===== Nodes =====
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # absolute path via gz_args
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={
            # format: "-r -v 1 <absolute_path_to_world>"
            'gz_args': [TextSubstitution(text='-r -v 1 '), world_file]
        }.items(),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Spawn entity at specified pose
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'tricycle',
            '-allow_renaming', 'true',
            '-x', x_pose, '-y', y_pose, '-z', z_pose,
            '-R', roll, '-P', pitch, '-Y', yaw
        ],
    )

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', robot_controllers],
        output='screen'
    )

    tricycle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'tricycle_controller',
            '--param-file', robot_controllers,
            '--ros-args', '-r', '/tricycle_controller/cmd_vel:=/cmd_vel'
        ],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(ekf_params_path, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
             ]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Events: load controllers after entity spawned
    after_spawn_start_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    after_jsb_start_tricycle = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[tricycle_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.00'),
        DeclareLaunchArgument('y_pose', default_value='-0.50'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        DeclareLaunchArgument('roll',   default_value='0.00'),
        DeclareLaunchArgument('pitch',  default_value='0.00'),
        DeclareLaunchArgument('yaw',    default_value='0.00'),

        gz_sim_launch,
        gz_bridge,
        ekf_node,
        node_robot_state_publisher,
        gz_spawn_entity,
        after_spawn_start_jsb,
        after_jsb_start_tricycle,
        # rviz2,
    ])
