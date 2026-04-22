## 개발환경
- OS : **Ubuntu 22.04/Ubunt 24.04**
- ROS Version : **ROS2 Humble / ROS Jazzy**
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

#### 1. nar_amr_control Package
**1.1 Package Structure**
```
bash
jinhan@jinhan:~/nar_amr/src/nar_amr_control$ tree
.
├── CMakeLists.txt
├── config
│   └── real_amr_control.yaml
├── include
│   └── nar_amr_control
├── launch
│   ├── real_robot.launch.py
│   └── sim_all.launch.py
├── package.xml
├── scripts
│   ├── integrated_swerve_controller.py
│   ├── sim_swerve_ik.py
│   └── swerve_gui.py
└── src

7 directories, 8 files
```
**1.2 Package Files Summary**

| 파일 | 역할 | 주요 입력 | 주요 출력 / 동작 | 비고 |
|---|---|---|---|---|
| `scripts/integrated_swerve_controller.py` | 실제 로봇용 **메인 통합 제어 노드** | `/cmd_vel` 또는 launch로 지정한 `cmd_vel_topic`, `/manual_steer_override`, `/can0/from_can_bus`, `/can1/from_can_bus`, YAML 파라미터 | 스워브 IK 계산, 조향/구동 CAN 명령 전송, `/drive_rpms`, `/steer_angles`, `/joint_states`, `/odom_raw` 발행, watchdog, SYNC heartbeat, startup/shutdown 처리 | 단순 IK 노드가 아니라 **실기 운용 전체를 담당하는 통합 컨트롤러** |
| `scripts/sim_swerve_ik.py` | 시뮬레이션용 **스워브 IK + 오도메트리 노드** | `/cmd_vel`, `/joint_states` | `/steering_controller/commands`, `/drive_controller/commands`, `/odom_raw` 발행 | 실제 조인트 상태를 다시 읽어 **폐루프 기반 오도메트리** 계산 |
| `scripts/swerve_gui.py` | 원격 조작 및 테스트용 **Tkinter GUI** | 사용자 버튼 입력 | `/cmd_vel`, `/manual_steer_override` 발행 | Teleop 탭 + Steer Calibration 탭 제공 |
| `launch/real_robot.launch.py` | 실제 로봇 실행용 런치 파일 | `params_file`, `cmd_vel_topic` launch argument | `ros2_socketcan` sender/receiver를 `can0`, `can1`에 실행하고 `integrated_swerve_controller.py` 구동 | 실기 CAN 브링업 + 메인 제어 노드 실행 진입점 |
| `launch/sim_all.launch.py` | 시뮬레이션 테스트용 런치 파일 | 별도 주요 입력 없음 | `sim_swerve_ik.py`, `swerve_gui.py`를 `use_sim_time=True`로 실행 | Gazebo 전체 실행이라기보다 **시뮬 제어 노드 + GUI 실행 묶음** |
| `config/real_amr_control.yaml` | 실제 로봇용 **기본 파라미터 파일** | `integrated_swerve_controller`가 로드 | 로봇 geometry, 속도 제한, drive/steer motor ID, bus mapping, watchdog, coordinated control, steering gate, startup support task 설정 | 현재 실기 기준 파라미터가 모여 있는 핵심 설정 파일 |

> Note: `swerve_gui.py`의 Steer Calibration 기능은 `/manual_steer_override`를 발행하며, 현재 구조상 실기용 `integrated_swerve_controller.py`와 직접 연결되는 기능.

#### 2. nar_amr_description Package
**2.1 Package Structure**
```
bash
jinhan@jinhan:~/nar_amr/src/nar_amr_description$ tree
.
├── CMakeLists.txt
├── config
│   └── swerve_controllers.yaml
├── include
│   └── nar_amr_description
├── launch
│   ├── sim.launch.py
│   ├── swerve_gui.py
│   └── test.launch.py
├── meshes
│   ├── body_link.stl
│   ├── caster_bracket.stl
│   ├── caster_wheel.stl
│   ├── drive_motor.stl
│   ├── rplidar-s2-model-3d-stl.stl
│   ├── steering_base.stl
│   └── steering.stl
├── package.xml
├── src
└── urdf
    └── nar_amr.urdf.xacro

8 directories, 14 files
```
**2.3URDF 구조**
<img width="1256" height="407" alt="image (2)" src="https://github.com/user-attachments/assets/21e10873-496b-4bab-8cb2-328f454c9884" />

#### 3. nar_amr_navigation Package
**3.1 Package Structure**
```
jinhan@jinhan:~/nar_amr/src/nar_amr_navigation$ tree
.
├── CMakeLists.txt
├── config
│   ├── ekf.yaml
│   ├── laser_filters.yaml
│   ├── mapper_params_sim.yaml
│   └── nav2_params.yaml
├── include
│   └── nar_amr_navigation
├── launch
│   ├── bringup.launch.py
│   ├── laser_filter.launch.py
│   ├── real_lidars.launch.py
│   ├── scan_merger.launch.py
│   └── slam.launch.py
├── maps
│   ├── factory_map.pgm
│   └── factory_map.yaml
├── package.xml
├── scripts
│   ├── gui_crab_walk.py
│   ├── test_crab_walk.py
│   └── waypoint_nav.py
└── src

8 directories, 16 files
```
> scripts 디렉토리 내 파이썬 파일은 Gazebo Simulator에서 단순히 테스트 용도로 만든 것임.

**3.2 Package Files Summary**



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







