import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav_pkg_share = get_package_share_directory('nar_amr_navigation')
    
    # 런타임에 use_sim_time을 결정할 수 있도록 인자 추가 (기본값 false)
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    # 파라미터 파일 경로 (실제 구동용 yaml을 바라보도록 설정)
    slam_config_path = os.path.join(nav_pkg_share, 'config', 'mapper_params_sim.yaml')

    # 공식 런처 정의
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'params_file': slam_config_path,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        GroupAction(
            actions=[
                SetRemap(src='/scan', dst='/scan'),
                slam_launch
            ]
        )
    ])