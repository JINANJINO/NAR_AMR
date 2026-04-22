import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    s2_baudrate = 1000000
    
    # 1. Front LiDAR Node
    front_lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_front',
        parameters=[{
            'serial_port': '/dev/rplidar_front', # scan lidar 고정 포트 이름
            'serial_baudrate': s2_baudrate,
            'frame_id': 'laser_front',
            'inverted': True,
            'angle_compensate': True,
        }],
        remappings=[('scan', 'scan_front')],
        output='screen'
    )
    
    # 2. Back LiDAR Node
    back_lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_back',
        parameters=[{
            'serial_port': '/dev/rplidar_back',   # back lidar 고정 포트 이름
            'serial_baudrate': s2_baudrate,
            'frame_id': 'laser_back',             
            'inverted': True,
            'angle_compensate': True,
        }],
        remappings=[('scan', 'scan_back')],
        output='screen'
    )
    
    return LaunchDescription([
        front_lidar_node,
        back_lidar_node
    ])
