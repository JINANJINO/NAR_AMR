from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('nar_amr_control')
    socketcan_share = get_package_share_directory('ros2_socketcan')

    params_file = LaunchConfiguration('params_file')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    can0_nodes = GroupAction([
        PushRosNamespace('can0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([socketcan_share, 'launch', 'socket_can_sender.launch.py'])
            ),
            launch_arguments={
                'interface': 'can0',
                'to_can_bus_topic': 'to_can_bus',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([socketcan_share, 'launch', 'socket_can_receiver.launch.py'])
            ),
            launch_arguments={
                'interface': 'can0',
                'from_can_bus_topic': 'from_can_bus',
            }.items(),
        ),
    ])

    can1_nodes = GroupAction([
        PushRosNamespace('can1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([socketcan_share, 'launch', 'socket_can_sender.launch.py'])
            ),
            launch_arguments={
                'interface': 'can1',
                'to_can_bus_topic': 'to_can_bus',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([socketcan_share, 'launch', 'socket_can_receiver.launch.py'])
            ),
            launch_arguments={
                'interface': 'can1',
                'from_can_bus_topic': 'from_can_bus',
            }.items(),
        ),
    ])

    swerve_node = Node(
        package='nar_amr_control',
        executable='integrated_swerve_controller.py',
        name='integrated_swerve_controller',
        output='screen',
        parameters=[
            params_file,
            {
                'cmd_vel_topic': cmd_vel_topic,
                'to_can_bus_topic_can0': '/can0/to_can_bus',
                'to_can_bus_topic_can1': '/can1/to_can_bus',
                'from_can_bus_topic_can0': '/can0/from_can_bus',
                'from_can_bus_topic_can1': '/can1/from_can_bus',
            }
        ],
    )

    default_swerve_params = PathJoinSubstitution([
        pkg_share,
        'config',
        'real_amr_control.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_swerve_params),
        DeclareLaunchArgument('cmd_vel_topic', default_value=TextSubstitution(text='/cmd_vel')),
        can0_nodes,
        can1_nodes,
        swerve_node,
    ])