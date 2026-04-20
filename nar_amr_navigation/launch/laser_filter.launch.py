import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_dir = os.path.join(get_package_share_directory('nar_amr_navigation'), 'config')
    filter_config = os.path.join(config_dir, 'laser_filters.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_front_filter',
            parameters=[filter_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('scan', 'scan_front'),
                ('scan_filtered', 'scan_front_filtered')
            ],
            output='screen'
        ),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_back_filter',
            parameters=[filter_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('scan', 'scan_back'),
                ('scan_filtered', 'scan_back_filtered')
            ],
            output='screen'
        ),
    ])