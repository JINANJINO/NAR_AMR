## 개발환경
- OS : **Ubuntu 22.04**
- ROS Version : **ROS2 Humble**
- Simulator : **Gazebo**

---

### Package별 역할

 Package | 분류 | 주요 역할 | 주요 구성 / 비고 |
|---|---|---|---|
| `nar_amr_control` | 실기 제어 | 실제 AMR 구동용 제어 패키지 | `integrated_swerve_controller.py`, `real_robot.launch.py`, `swerve_gui.py`, `sim_swerve_ik.py` 포함. 듀얼 CAN 기반 스워브 통합 제어, `/cmd_vel` 입력, GUI 연동, 실기 bringup 담당 |
| `nar_amr_description` | 모델/URDF | 로봇 모델 정의 및 시각화 패키지 | `urdf/nar_amr.urdf.xacro`, `meshes/`, `config/swerve_controllers.yaml`, `sim/test` launch 포함 |
| `nar_amr_navigation` | 내비게이션 | EKF, SLAM, Nav2, 라이다 전처리 launch 제공 | `ekf.yaml`, `nav2_params.yaml`, `mapper_params_sim.yaml`, `laser_filter.launch.py`, `slam.launch.py`, `bringup.launch.py` 등 포함 |
| `nar_amr_gazebo` | 시뮬레이션 | Gazebo 월드 및 시뮬레이션 환경 패키지 | `worlds/factory.sdf` 중심의 Gazebo 환경 구성 |
| `ros2_laser_scan_merger` | 센서 융합 | 다중 라이다 스캔 병합 패키지 | `main.cpp` 기반 C++ 노드, `merge_2_scan.launch.py`, RViz 설정 포함 |
| `rplidar_ros` | 센서 드라이버 | RPLIDAR ROS2 드라이버 | 라이다 실기 입력 담당 패키지 |
| `bluespace_ai_xsens_ros_mti_driver` | 센서 드라이버 | Xsens MTi IMU 드라이버 패키지 | IMU 데이터 수집/퍼블리시 담당으로 사용 |

---

### Simulator 실행 명령어 정리

- **Gazebo & Gui node & Rviz 실행 명령어**

  ```
  source ./install/local_setup.bash 
  ros2 launch nar_amr_navigation bringup.launch.py use_sim_time:=true
  ```

- **Lidar sensor fusion 실행**

  ```
  ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py use_sim_time:=true
  ```

- **역기구학 연산을 통해 4개 바퀴의 개별 조향각과 구동 속도로 번역하여 시뮬레이터에 전달하는 핵심 제어 모듈 실행**

  ```
  ros2 run nar_amr_control sim_swerve_ik.py --ros-args -p use_sim_time:=true
  ```

- **지정한 정적 지도(Map)와 주행 설정(Params)을 시스템에 주입하여 로봇의 위치 추정 및 자율주행 핵심 기능을 일괄 활성화**

  ```
  ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/home/jinhan/nar_amr/src/nar_amr_navigation/maps/factory_map.yaml params_file:=/home/jinhan/nar_amr/src/nar_amr_navigation/config/nav2_params.yaml
  ```

---

### Simulator 실행 모습
[Screencast from 2026-04-08 16-21-38.webm](https://github.com/user-attachments/assets/47a83e21-6cba-454d-b6bb-9f661333b248)

---

### Lidar data, 실제 모터 구동 확인
#### Rviz 내 Lidar data 확인(전처리 전)
<img width="664" height="439" alt="Screenshot from 2026-04-22 09-24-39" src="https://github.com/user-attachments/assets/cbc963d5-4210-46db-b542-5b4d756a36c1" />


#### 실제 모터 구동 테스트
<img width="400" alt="20260421_162241-ezgif com-cut" src="https://github.com/user-attachments/assets/a3883226-f4b5-4e20-a4d2-13baed32f527" />







