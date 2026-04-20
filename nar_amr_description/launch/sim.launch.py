import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node, SetParameter 
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('nar_amr_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'nar_amr.urdf.xacro')
    
    mesh_path = os.path.join(pkg_share, '..')
    set_gazebo_model_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', mesh_path)
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 1. 로봇 상태 퍼블리셔
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'publish_frequency': 100.0, 'use_sim_time': True }]
    )

    # 2. Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f"-r {os.path.join(get_package_share_directory('nar_amr_gazebo'), 'worlds', 'factory.sdf')}"}.items(), 
    )

    # 3. 로봇 스폰
    spawn_entity = Node(package='ros_gz_sim', executable='create', output='screen',
                        arguments=['-topic', 'robot_description', '-name', 'neo1_amr', '-z', '0.5'])

    # 4. 컨트롤러 스포너
    load_joint_state_broadcaster = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    load_steering_controller = Node(package="controller_manager", executable="spawner", arguments=["steering_controller"])
    load_drive_controller = Node(package="controller_manager", executable="spawner", arguments=["drive_controller"])

    # 5. Jazzy용 Gazebo ↔ ROS 2 브릿지 
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/factory_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',             
            '/scan_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan_back@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU 브릿지 추가 (Jazzy 표준 경로)
            '/model/neo1_amr/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Gazebo 오도메트리 (충돌 방지를 위해 이름을 바꿈)
            '/model/neo1_amr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/neo1_amr/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            ('/world/factory_world/clock', '/clock'),
            ('/model/neo1_amr/imu', '/imu/data'), # IMU 데이터를 EKF가 쓰는 이름으로 변경
            ('/model/neo1_amr/odometry', '/odom_gz'), # EKF의 결과물(/odom)과 겹치지 않게 변경
            ('/model/neo1_amr/tf', '/tf_gz') # Gazebo의 가짜 TF가 EKF를 방해하지 못하게 변경
        ],
        output='screen'
    )

    # 6. 이름표 어댑터
    node_adapter_front = Node(package='tf2_ros', executable='static_transform_publisher',
                              arguments=['0', '0', '0', '0', '0', '0', 'laser_front', 'neo1_amr/base_footprint/front_lidar'])
    node_adapter_back = Node(package='tf2_ros', executable='static_transform_publisher',
                             arguments=['0', '0', '0', '0', '0', '0', 'laser_back', 'neo1_amr/base_footprint/back_lidar'])


    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        set_gazebo_model_path,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_steering_controller, load_drive_controller])),
        node_ros_gz_bridge,
        node_adapter_front,
        node_adapter_back
    ])