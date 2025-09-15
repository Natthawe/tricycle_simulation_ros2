from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    urdf_path = FindPackageShare('antbot_description')
    rviz_path = PathJoinSubstitution([urdf_path, 'rviz', 'jsp.rviz'])

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    
    rviz_arg = DeclareLaunchArgument(name='rviz', default_value=rviz_path,
                                    description='Absolute path to rviz config file')
    
    model_arg = DeclareLaunchArgument(
        'model', default_value='tricycle_drive.xacro.urdf',
        description='Name of the URDF description to load'
    )

    urdf = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'antbot_description',
            'urdf_package_path': PathJoinSubstitution(['urdf', 'robots', LaunchConfiguration('model')]),
            'rviz_config': LaunchConfiguration('rviz'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(gui_arg)
    launchDescriptionObject.add_action(rviz_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(urdf)

    return launchDescriptionObject
