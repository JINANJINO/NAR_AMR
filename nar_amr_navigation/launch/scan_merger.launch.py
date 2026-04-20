import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    merger_pkg = get_package_share_directory('ros2_laser_scan_merger')
    config = os.path.join(merger_pkg, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            name='ros2_laser_scan_merger',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cloud_in', '/cloud_in'),
                ('scan', '/scan')
            ]
        )
    ])