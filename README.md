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
| `scripts/swerve_gui.py` | 원격 조작 및 테스트용 **Tkinter GUI** | 사용자 버튼 입력 | `/cmd_vel`(Teleop 시 **버튼을 누르고 있는 동안** 약 20 Hz 반복), `/manual_steer_override` 발행 | Teleop 탭 + Steer Calibration 탭, 버튼을 떼면 정지 |
| `launch/real_robot.launch.py` | 실제 로봇 실행용 런치 파일 | `params_file`, `cmd_vel_topic` launch argument | `ros2_socketcan` sender/receiver를 `can0`, `can1`에 실행하고 `integrated_swerve_controller.py` 구동 | 실기 CAN 브링업 + 메인 제어 노드 실행 진입점 |
| `launch/sim_all.launch.py` | 시뮬레이션 테스트용 런치 파일 | 별도 주요 입력 없음 | `sim_swerve_ik.py`, `swerve_gui.py`를 `use_sim_time=True`로 실행 | Gazebo 전체 실행이라기보다 **시뮬 제어 노드 + GUI 실행 묶음** |
| `config/real_amr_control.yaml` | 실제 로봇용 **기본 파라미터 파일** | `integrated_swerve_controller`가 로드 | 로봇 geometry, 속도 제한, drive/steer motor ID, bus mapping, watchdog, coordinated control, steering gate, startup support task 설정 | 현재 실기 기준 파라미터가 모여 있는 핵심 설정 파일 |

> Note: `swerve_gui.py`의 Steer calibration은 `/manual_steer_override`만 발행하며 조향 위주로 동작합니다. Teleop 탭은 `/cmd_vel`을 발행합니다. CAN **heartbeat** 워치독이 걸린 동안은 컨트롤러가 `/cmd_vel`을 무시합니다.

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
**2.2 Package Files Summary**
| 구분 | 파일 | 요약 |
|---|---|---|
| Launch | `launch/sim.launch.py` | Xacro 기반 `robot_description`을 publish하고, Gazebo Sim의 `factory.sdf` 월드를 실행한 뒤 로봇을 `neo1_amr`로 spawn. 이후 `joint_state_broadcaster`, `steering_controller`, `drive_controller`를 순차적으로 실행하고, `/clock`, `/scan_front`, `/scan_back`, `/imu/data`, `/odom_gz`, `/tf_gz` 브릿지를 구성. |
| Launch | `launch/swerve_gui.py` | `/steering_controller/commands`와 `/drive_controller/commands`에 직접 명령을 보내는 저수준 테스트 GUI이다. 직진, 후진, 좌/우 게걸음, 제자리 회전, 정지 프리셋을 버튼으로 보낼 수 있다. |
| Launch | `launch/test.launch.py` | Xacro를 URDF로 변환하여 `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`를 실행하는 모델 검증용 런치 파일. Gazebo 없이 링크/조인트 구조와 외형을 빠르게 확인할 때 사용. |
| Config | `config/swerve_controllers.yaml` | `controller_manager`를 100 Hz로 설정, `joint_state_broadcaster`, `steering_controller`(4개 steering joint position control), `drive_controller`(4개 wheel joint velocity control)를 정의. |
| URDF | `urdf/nar_amr.urdf.xacro` | `base_footprint`-`body_link` 본체 구조 위에 4개의 스워브 모듈(FL/FR/RL/RR), 2개의 캐스터(front/rear), 전·후방 LiDAR 링크를 정의. 시뮬레이션 모드에서는 `ros2_control` 인터페이스, Gazebo 센서(IMU, front/back LiDAR), Gazebo odometry publisher, `gz_ros2_control` 플러그인을 추가


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

| 구분 | 파일 | 요약 |
|---|---|---|
| Launch | `bringup.launch.py` | 실기/시뮬레이션 공통 상위 bringup |
| Launch | `laser_filter.launch.py` | 전·후방 LiDAR 필터링 |
| Launch | `real_lidars.launch.py` | 실기 RPLidar 2대 실행 |
| Launch | `scan_merger.launch.py` | 다중 스캔 병합 후 `/scan` 생성 |
| Launch | `slam.launch.py` | SLAM Toolbox 실행 |
| Config | `ekf.yaml` | `/odom_raw + imu/data -> /odom` EKF 융합 |
| Config | `laser_filters.yaml` | 차체 영역 LiDAR 제거 필터 |
| Config | `mapper_params_sim.yaml` | SLAM Toolbox 파라미터 |
| Config | `nav2_params.yaml` | Nav2 전체 파라미터 |
| Map | `factory_map.yaml` | 정적 지도 메타파일 |
| Map | `factory_map.pgm` | 정적 지도 이미지 |


---

### 실제 로봇 실행 명령어 정리

워크스페이스를 빌드한 뒤 터미널에서 아래를 먼저 실행합니다.

```bash
source ./install/setup.bash
```

#### 제어(CAN·스워브)만 실행할 때

시뮬레이터 없이 구동만 확인하거나, 내비 패키지와 분리해 켤 때 사용합니다.

```bash
ros2 launch nar_amr_control real_robot.launch.py
```

- `ros2_socketcan`으로 `can0` / `can1` 송수신과 `integrated_swerve_controller`가 함께 올라갑니다.
- Nav2·SLAM은 포함되지 않습니다.

#### 텔레옵 (`real_robot`만 켠 뒤)

`integrated_swerve_controller`는 기본으로 `/cmd_vel`(`geometry_msgs/msg/Twist`)을 구독합니다. `real_robot.launch.py`를 켠 상태에서 **다른 터미널**에서 속도 명령을 내면 됩니다.

**키보드 (`teleop_twist_keyboard`)**

```bash
sudo apt install ros-humble-teleop-twist-keyboard
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- ROS 2 Jazzy를 쓰면 패키지 이름을 `ros-jazzy-teleop-twist-keyboard`로 바꿉니다.

- 기본 발행 토픽은 `/cmd_vel`이라 remap 없이 동작합니다.
- 키 설명은 해당 터미널에 출력됩니다. **그 터미널 창에 포커스**가 있어야 입력이 들어갑니다.

다른 토픽 이름을 쓰려면(예: 안전 레이어 뒤의 `/cmd_vel_safe`) 양쪽을 맞춥니다.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_safe
```

```bash
ros2 launch nar_amr_control real_robot.launch.py cmd_vel_topic:=/cmd_vel_safe
```

**조이스틱 (`teleop_twist_joy`)**

```bash
sudo apt install ros-humble-teleop-twist-joy
```

`joy_node`와 `teleop_node`를 launch로 묶어 `/cmd_vel`로 보내는 구성을 쓰면 됩니다.

**이 레포의 GUI (`swerve_gui.py`)**

Tkinter 기반 패널로 `/cmd_vel`과 `/manual_steer_override`를 발행합니다.

```bash
ros2 run nar_amr_control swerve_gui.py
```

- **실행 환경:** 로봇 PC 또는 WSL이 아니라 **그래픽 디스플레이가 있는 환경**에서 실행하세요. WSL은 X11/WSLg 등으로 GUI 창이 떠야 합니다. `integrated_swerve_controller`는 **로봇 PC에서** `real_robot.launch.py` 등과 함께 띄우는 것이 일반적입니다(원격이면 ROS_DOMAIN_ID·네트워크를 맞춤).
- **탭 구성**
  - **Teleop control:** 이동 방향 버튼을 **누르고 있는 동안만** `/cmd_vel`을 약 20 Hz로 발행합니다. 버튼을 떼면 자동으로 정지(`Twist` 0)합니다. **STOP**은 즉시 정지, 탭 전환 시에도 정지합니다.
  - **Steer calibration:** 각 바퀴 각도(도) 조정 후 **PUBLISH OVERRIDE**로 `/manual_steer_override`를 보냅니다(전송 전 Teleop 정지). 이 토픽은 **조향만** 적용하고 구동은 0입니다. 주행은 Teleop 탭을 쓰세요.
- **속도 버튼:** `+ Lin Vel` 등은 **버튼을 누른 채 이동 중일 때**만 즉시 반영됩니다.

**주의 (컨트롤러 안전 기능 — 서로 다름)**

1. **CAN heartbeat 워치독** (`can_heartbeat_timeout_sec`, 기본 3초): 모터 **heartbeat CAN**(COB-ID `0x701`~`0x77F`)이 끊기면 비상 정지 후 `/cmd_vel`·`/manual_steer_override` 무시. 로그에 **CAN 통신 점검** 안내. heartbeat가 다시 오면 자동 해제.
2. **ROS cmd_vel 타임아웃** (`cmd_vel_timeout_sec`, 기본 0.5초): `/cmd_vel`이 일정 시간 없으면 구동 모터만 정지(GUI/텔레옵 단절 대비). heartbeat 워치독과 별개입니다.
3. **manual_steer 후 cmd_vel 억제** (`cmd_vel_suppress_after_manual_steer_sec`, 기본 2초): 조향 캘리브레이션 직후 남은 `/cmd_vel` 무시.

- 벤치에서 heartbeat 없이 시험하려면 [real_amr_control.yaml](nar_amr_control/config/real_amr_control.yaml)의 `can_heartbeat_timeout_sec`를 `0`으로 두는 방법이 있으나, **실차에서는 비권장**입니다.

#### 전체 bringup (`nar_amr_navigation`)

센서 전처리, EKF, RViz 등을 한 번에 올리는 진입점입니다.

```bash
ros2 launch nar_amr_navigation bringup.launch.py use_sim_time:=false
```

- `use_sim_time:=false`일 때: `real_robot.launch.py`(CAN + 스워브), `real_lidars.launch.py`(전·후방 RPLidar), `laser_filter.launch.py`, EKF(`ekf.yaml`), RViz2가 포함됩니다.
- **주의:** 현재 `bringup.launch.py`는 `robot_state_publisher`용으로 `nar_amr_description`의 `sim.launch.py`를 항상 포함합니다. 그 안에서 Gazebo 등 시뮬 관련 프로세스가 함께 시작될 수 있으니, 시뮬 없이 실기만 쓰려면 위의 `real_robot.launch.py`만 켜거나 bringup 구성을 실기 전용으로 정리하는 것이 좋습니다.

#### IMU (EKF용, 선택)

[ekf.yaml](nar_amr_navigation/config/ekf.yaml)은 `imu/data`를 구독합니다. Xsens MTi 등 실기 IMU 드라이버를 별도 터미널에서 실행하고, 토픽 이름이 `imu/data`와 맞는지 확인하세요. 설치된 패키지의 launch 이름은 버전마다 다를 수 있습니다.

```bash
# 예시 (패키지·파일명은 설치본에 맞게 수정)
ros2 launch bluespace_ai_xsens_ros_mti_driver xsens_mti_node.launch.py
```

#### 라이다 스캔 병합 (`/scan` 생성)

`bringup.launch.py` 안의 스캔 병합기 include는 주석 처리되어 있어, SLAM·Nav2 전에 **별도 터미널**에서 실행하는 것을 권장합니다.

```bash
ros2 launch nar_amr_navigation scan_merger.launch.py use_sim_time:=false
```

#### SLAM (온라인 매핑)

스캔 병합 후, 또 다른 터미널에서:

```bash
ros2 launch nar_amr_navigation slam.launch.py use_sim_time:=false
```

- SLAM Toolbox 설정은 [mapper_params_sim.yaml](nar_amr_navigation/config/mapper_params_sim.yaml)을 사용합니다. 실기에 맞게 조정할 수 있습니다.
- 저장한 맵은 `ros2 run slam_toolbox map_saver_cli` 등으로 `.yaml` / `.pgm` 파일로 저장할 수 있습니다.

#### Navigation (정적 맵 + Nav2)

미리 만든 맵과 Nav2 파라미터를 지정합니다. 경로는 워크스페이스에 맞게 바꾸거나, 설치 prefix를 사용할 수 있습니다.

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=false \
  map:=$(ros2 pkg prefix nar_amr_navigation)/share/nar_amr_navigation/maps/factory_map.yaml \
  params_file:=$(ros2 pkg prefix nar_amr_navigation)/share/nar_amr_navigation/config/nav2_params.yaml
```

- `factory_map.yaml` 대신 SLAM으로 저장한 맵 경로를 넣으면 됩니다.
- `scan_merger`로 `/scan`이 떠 있는 상태에서 실행하는 것을 권장합니다.

#### 실행 순서 요약 (실기 + SLAM 또는 네비)

1. (선택) IMU 드라이버  
2. `bringup.launch.py` **또는** 제어만 `real_robot.launch.py` + 필요 시 `real_lidars.launch.py` 등 수동 조합  
3. `scan_merger.launch.py`  
4. `slam.launch.py` **또는** `nav2_bringup`의 `bringup_launch.py`

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







