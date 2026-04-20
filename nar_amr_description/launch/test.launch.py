import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'nar_amr_description'
    file_subpath = 'urdf/nar_amr.urdf.xacro'

    # Xacro 파일 경로 찾기
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    
    # Xacro를 파싱해서 URDF(XML) 문자열로 변환
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 로봇 상태 퍼블리셔 노드 (URDF 정보를 ROS 시스템에 뿌려줌)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # 조인트 상태 퍼블리셔 GUI (마우스로 바퀴/조향축을 돌려볼 수 있는 슬라이더 창)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 노드 (3D 시각화 프로그램)
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])