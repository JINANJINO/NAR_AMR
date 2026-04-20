import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import SetParameter

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_laser_scan_merger'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        
        # 1. 병합기
        launch_ros.actions.Node(
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            parameters=[config],
            output='screen',
            respawn=True,
            respawn_delay=2,
            remappings=[('/base/custom_cloud', '/cloud_in')]
        ),

        # 2. 브릿지
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_laser',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'laser'
            ]
        ),

        # 3. 변환기 
        launch_ros.actions.Node(
            name='pointcloud_to_laserscan',
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            parameters=[config],
            output='screen',
            remappings=[
                ('/cloud_in', '/cloud_in'),
                ('/scan', '/scan')
            ]
        )
    ])