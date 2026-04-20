import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 경로 설정
    nav_pkg = get_package_share_directory('nar_amr_navigation')
    control_pkg = get_package_share_directory('nar_amr_control')
    desc_pkg = get_package_share_directory('nar_amr_description')
    
    # EKF 설정 파일 경로 추가
    ekf_config_path = os.path.join(nav_pkg, 'config', 'ekf.yaml')
    
    # 2. 실행 인자 정의 (기본값 true)
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # =========================================================================
    # [A] 공통 노드 레이어 (시뮬레이션/실제 로봇 모두 실행)
    # =========================================================================
    
    # 1. Robot State Publisher (URDF)
    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_pkg, 'launch', 'sim.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. 전처리 (필터 및 병합기)
    laser_processing = GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'laser_filter.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'scan_merger.launch.py')),
        #    launch_arguments={'use_sim_time': use_sim_time}.items()
        #),
    ])

    # 3. EKF 센서 퓨전 노드
    # 바퀴 엔코더와 IMU 데이터를 섞어 정밀한 위치(TF)를 발행합니다.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odom')] 
    )

    # 4. RViz2 시각화
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # =========================================================================
    # [B] 시뮬레이션 전용 레이어 (use_sim_time:=true 일 때)
    # =========================================================================
    simulation_env = GroupAction(
        condition=IfCondition(use_sim_time),
        actions=[
            # Gazebo 및 컨트롤러 실행
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(control_pkg, 'launch', 'sim_all.launch.py')),
                launch_arguments={'use_sim_time': 'true'}.items()
            ),
            
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='link_to_footprint_broadcaster_sim',
                arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
            ),
            
            # 라이다 좌표계 정렬용 스태틱 TF
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'laser_front', 'neo1_amr/base_footprint/front_lidar']
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'laser_back', 'neo1_amr/base_footprint/back_lidar']
            ),
        ]
    )

    # =========================================================================
    # [C] 실제 로봇 전용 레이어 (use_sim_time:=false 일 때)
    # =========================================================================
    real_hardware = GroupAction(
        condition=UnlessCondition(use_sim_time),
        actions=[
            # 실제 하드웨어 컨트롤러 실행
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(control_pkg, 'launch', 'real_robot.launch.py')),
                launch_arguments={'use_sim_time': 'false'}.items()
            ),
            # 실제 RPLidar S2 실행
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'real_lidars.launch.py'))
            ),
            
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='link_to_footprint_broadcaster_real',
                arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        simulation_env,
        real_hardware,
        robot_state_pub,
        laser_processing,
        ekf_node, # EKF 실행 리스트에 포함
        rviz_node
    ])