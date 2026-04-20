import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 시뮬레이션용 두뇌 (IK 노드) - 시뮬레이션 시간 동기화
    ik_node = Node(
        package='nar_amr_control',
        executable='sim_swerve_ik.py',
        name='sim_swerve_ik',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 2. 조종기 (GUI 노드) - 시뮬레이션 시간 동기화
    gui_node = Node(
        package='nar_amr_control',
        executable='swerve_gui.py',
        name='swerve_gui',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        ik_node,
        gui_node,
    ])