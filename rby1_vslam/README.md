# rby1_vslam

## UPC 로컬 오퍼레이터

조작 UI는 UPC의 Qt/RViz로만 실행한다. `physical.launch.py`는 수동 주행, 현재 위치
저장/삭제/불러오기, 웨이포인트 이동/취소와 TCP·추적·Nav2 상태를 보여 주는
`waypoint_ui`를 시작한다. 기존 웨이포인트 YAML을 그대로 사용한다.

```bash
colcon build --symlink-install --packages-up-to rby1_control rby1_vslam rby1_description
ros2 launch rby1_vslam physical.launch.py lab_host:=192.168.30.50
```

**랩 PC/TCP 연결 없이 수동 베이스만 움직일 때는** 프로젝트 루트에서
`bash run_mobile_base_upc.sh`를 실행한다. 이 경로는 카메라·VSLAM·Nav2를 실행하지 않으며
`lab_host`나 tracking 상태를 요구하지 않는다. 전체 VSLAM 실행은
`bash run_vslam_upc.sh lab_host:=<LAB-IP>`다. 두 스택을 동시에 실행하지 않는다.

UPC/LAB 지연을 정량적으로 기록하려면 **[양쪽 시간 계측 가이드](docs/timing_capture.md)**를
참고한다. 실행 로직을 바꾸지 않고 JSONL·rosbag 및 선택적 TCP pcap을 수집하여 오프라인
CSV/그래프 분석에 사용한다.

처음과 끝에 완전히 정지하고 mapping 경로를 돌아 시작점으로 복귀하는 시험은
**[mapping 왕복·정지 jitter 기록](docs/mapping_jitter_capture.md)**을 따른다. 대화형 phase
marker를 기준으로 tracking/SLAM/wheel pose의 정지 jitter와 시작↔복귀 오차, IMU 진동을
CSV와 PNG로 만든다.

근거리 waypoint의 Theta* path, MPPI 제어, status 6, localization 보정 점프와 실행 중
Nav2 parameter를 함께 조사할 때는 **[근거리 Nav2 진단/MPPI 적용](docs/nav2_close_goal_review.md)**를
따른다. UPC에서 `bash rby1_vslam/scripts/capture_navigation_debug.sh`를 실행하면 된다.

RBY1의 Jetson(Ubuntu 22.04 / ROS 2 Humble)에서 D435i를 읽고, LAB PC의 Ubuntu 24.04 / ROS 2 Jazzy / Isaac ROS 4.5로 TCP 전송해 VSLAM을 계산한다. 결과를 Jetson으로 돌려받아 로봇 베이스 위치로 변환하고, **Jetson의 Nav2**에서 경로를 계산한 뒤 기존 **`/rby1/cmd_raw` (`geometry_msgs/Twist`)**에 발행한다.

이 패키지만 양쪽에 복사해 빌드한다. **Jetson에는 Isaac ROS를 설치하지 않는다.** LAB PC만 Isaac ROS를 실행한다. 실물 전체 절차와 명령은 **[Jetson 실물 실행 가이드](docs/jetson_physical.md)**에 정리되어 있다.

**Nav2와 지도:** `navigation.launch.py`는 Theta* 경로계획 + MPPI `Omni` 메카넘 제어 + velocity smoother를 실행한다. 목표 허용오차는 위치 0.01 m, yaw 1°이며 두 조건과 정지 속도(0.02 m/s, 0.02 rad/s)를 동시에 판정한다. 코스트맵은 0.05 m 해상도이고 MPPI는 20 Hz, 0.05 s model step, 2.8 s 예측 지평선을 사용한다. cuVSLAM 특징점 지도와 Nav2의 occupancy map은 다른 데이터다. 실물 launch 기본값 `use_scan:=false`에서는 장애물 layer가 제거되며 회피 기능이 없다. LiDAR를 연결한 뒤 `use_scan:=true`로 실행하면 `/scan`으로 local/global obstacle layer를 사용한다. 같은 `vslam_map` 좌표로 정렬된 occupancy map이 있다면 `occupancy_map:=/절대경로/map.yaml` 옵션으로 map_server/static layer를 사용한다.

## 담당 범위

```text
UPC / Humble
  D435i 드라이버 ── stereo + CameraInfo + IMU + 카메라 내부 static TF
       │
  upc_bridge ───────── TCP 7447 ────────────────┐
       ▲                                      │
       │ camera odometry                      ▼
  pose_adapter                          LAB / Jazzy
       │ timestamp 시점 base↔d435_link TF   lab_bridge
       │                                      │
       │                                 Isaac ROS 4.5 / RTX 5080
       │                                 VIO + SLAM + map save/localize
       │                                      │
       └─────────────── TCP 결과 반환 ──────────┘
       │
  /rby1/vslam/odom       (연속 VIO 위치, vslam_odom)
  /rby1/vslam/slam_odom  (지도 보정 위치, vslam_map)
       │
  localization_tf: vslam_map → 기존 wheel odom 정렬
       │
  Nav2 (/rby1/odom 사용, use_scan=true일 때 /scan도 사용)
       │
  nav2_gate (기본 비활성, 입력 유효성 확인)
       │
  /rby1/cmd_raw → 기존 rby1_control → /rby1/cmd_vel → 기존 rby1_driver
```

로봇 구동·전원·서보·stream control은 기존 `rby1_control`과 드라이버 담당이다. `physical.launch.py`는 driver/control을 실행하지만 전원·서보·stream은 자동으로 켜지 않는다. UI에서 작업자가 명시적으로 준비하며, 실물 collision 상태는 절대로 우회하지 않는다. 다른 `cmd_raw` 발행기와 동시에 실행하지 않는다.

카메라가 **항상 같은 머리 자세여야 하는 것은 아니다.** cuVSLAM은 카메라에 고정된 `d435_link`를 추적하고, UPC는 촬영 시점의 실제 관절 TF로 `base` 위치를 계산한다. 따라서 정확한 관절 TF와 장착 보정이 있으면 머리·몸통 운동을 분리할 수 있다. 첫 검증 때만 자세를 고정해 전송/추적/TF 문제를 나눠 확인한다. TF가 없으면 임의의 고정 변환으로 대체하지 않는다.

## VSLAM 카메라 고정 마운트 확인

VSLAM이 사용하는 장착 TF는 [`launch/upc.launch.py`](launch/upc.launch.py)에 있다.
두 변환을 합친 `link_head_2 → d435_link`가 카메라의 실제 장착 자세다.

```text
link_head_2
  └─ d435_camera_center  xyz=(0.0464, 0, 0.066), rpy=(0, -2 deg, 0)
       └─ d435_link      xyz=(0.02185, 0.0175, 0), rpy=(0, 0, 0)
```

- 첫 번째 변환은 `mount_x`, `mount_y`, `mount_z`, `mount_roll`, `mount_pitch`,
  `mount_yaw` launch argument다. 기본값은
  [`launch/physical.launch.py`](launch/physical.launch.py)에서도 확인할 수 있다.
- 두 번째 변환은 `upc.launch.py`의 `d435_vslam_center` static transform이다.
- 값의 출처는
  [`../rby1_camera/config/camera_system.yaml`](../rby1_camera/config/camera_system.yaml)의
  `D435 장착 TF` 항목이다. 이 파일에도 실측 전 초기값이라고 표시되어 있다.
- 다른 노드가 동일 TF를 이미 발행한다면 `publish_mount_tf:=false`로 중복 authority를
  막는다. 그렇지 않으면 기본값 `true`를 유지한다.

실측값은 코드를 바꾸지 않고 실행 시 덮어쓸 수 있다. 각도 단위는 radian이다.

```bash
ros2 launch rby1_vslam physical.launch.py \
  lab_host:=192.168.30.50 \
  mount_x:=0.0464 mount_y:=0.0 mount_z:=0.066 \
  mount_roll:=0.0 mount_pitch:=-0.034906585 mount_yaw:=0.0
```

실행 중에는 다음 두 명령으로 관절 자세와 장착값이 합쳐진 최종 TF를 확인한다.

```bash
ros2 run tf2_ros tf2_echo link_head_2 d435_link
ros2 run tf2_ros tf2_echo base d435_link
```

## 파일과 실행 위치

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `physical.launch.py` | Jetson | 실물 driver + TF + control + camera/TCP + Nav2 + RViz/UI |
| `d435i.launch.py` | Jetson | D435i 1대만 실행, 기존 `/d435/d435` 이름 |
| `upc.launch.py` | Jetson | 카메라 + 장착 TF + TCP client + pose adapter |
| `lab.launch.py` | LAB Jazzy | TCP server + cuVSLAM, `mode` 선택 |
| `mapping.launch.py` | LAB Jazzy | 새 특징점 지도 작성 |
| `localization.launch.py` | LAB Jazzy | 기존 지도 선택, 이후 위치 힌트로 localize |
| `navigation.launch.py` | Jetson | Nav2 + VSLAM 위치 TF + cmd_raw 게이트 |
| `operator_qt.launch.py` | Jetson | RViz + 수동 mapping + named Point UI |
| `mobile_base_upc.launch.py` | Jetson | driver/control + 로컬 수동 이동 Qt |
| `map_tool` | LAB Jazzy | 지도 저장 / 힌트 기반 재위치 추정 |

`upc.launch.py`가 내부에서 `d435i.launch.py`를 포함한다. 따라서 `d435i.launch.py`를
별도로 먼저 실행했다면 UPC launch에 `start_camera:=false`를 주어 중복을 막는다.
mapping/localization/lab 중 하나만 LAB에서 실행한다.

### LAB/UPC 빠른 실행 스크립트

반복 실험에서는 scripts/run_vslam_lab.sh와 scripts/run_vslam_upc.sh를 사용한다.
LAB 스크립트는 Isaac ROS Jazzy 컨테이너 내부에서 cuVSLAM과 RViz2를 함께 띄우며,
좌우 IR 영상, TF, odometry, VO/SLAM 경로와 pose graph가 등록된 전용 RViz preset을
Fixed Frame `vslam_map`으로 시작한다. 기본값은 mapping, IMU 활성,
ROS_DOMAIN_ID=85이다.

    # LAB PC의 Isaac ROS 컨테이너 내부
    cd /workspaces/isaac_ros-dev/src/rby1_vslam
    ./scripts/run_vslam_lab.sh

UPC 스크립트는 Jetson Humble에서 실행하고 LAB PC의 실제 유선 IP를 전달한다.

    cd "$HOME/rby1_ros2_ws/src/rby1-project/rby1_vslam"
    ./scripts/run_vslam_upc.sh --lab-host 192.168.30.50

UPC 스크립트는 upc.launch.py만 실행하므로 robot driver, control, Nav2와 operator UI는
시작하지 않는다. 전체 실물 stack은 physical.launch.py를 사용하고, 수동 mapping용
driver/control/UI 묶음은 scripts/run_vslam_mapping_upc.sh를 사용한다.

## 오늘 실물 실험 순서

아래 순서는 Isaac Sim을 사용하지 않는다. Jetson은 ROS 2 Humble과 실물 RBY1/D435i만
실행하고, LAB PC의 Isaac ROS 컨테이너가 cuVSLAM만 계산한다.

### 1. LAB PC 유선 IP 확인

LAB PC 호스트에서 확인하고 Jetson에서 ping한다. 아래부터 `192.168.30.50`은 실제 LAB
유선 IP로 교체한다. `127.0.0.1`과 `172.17.x.x` Docker 주소는 사용하지 않는다.

```bash
# LAB PC
ip -4 -br addr

# Jetson
ping -c 3 192.168.30.50
```

### 2. Jetson 빌드

Jetson의 기존 `rby1_ros2_ws/src/rby1-project`에서 빌드한다. 사용자명이
`nvidia-tegra`인지 추측해서 경로에 넣지 않고 `$HOME`을 사용한다.

```bash
export RBY1_WS="$HOME/rby1_ros2_ws"
export RBY1_SDK_PATH="$HOME/sdk/rby1-sdk"  # 실제 SDK 위치로 수정
export LD_LIBRARY_PATH="$RBY1_SDK_PATH/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source /opt/ros/humble/setup.bash
cd "$RBY1_WS"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-up-to \
  rby1_driver rby1_description rby1_control rby1_vslam
source install/setup.bash
```

### 3. LAB PC Isaac ROS 컨테이너 빌드

LAB workspace에 같은 `rby1_vslam` 소스를 둔다. 호스트에서 `ISAAC_ROS_WS`를 먼저
지정해야 `isaac-ros activate`가 workspace를 찾는다.

```bash
# LAB 호스트
export ISAAC_ROS_WS="$HOME/workspaces/isaac_ros-dev"
test -d "$ISAAC_ROS_WS/src/rby1_vslam"
isaac-ros activate
```

위 명령으로 들어간 Isaac ROS Jazzy 컨테이너에서 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/isaac_ros-dev
rosdep install --from-paths src/rby1_vslam --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-select rby1_vslam
source install/setup.bash
ros2 pkg prefix isaac_ros_visual_slam
```

더 자세한 최초 설치와 문제 해결은
[Jetson 실물 실행 가이드](docs/jetson_physical.md)를 참고한다.

### 4. Mapping 실행

먼저 **Terminal 1 — LAB PC Isaac ROS 컨테이너**에서 서버를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=85
ros2 launch rby1_vslam mapping.launch.py \
  bind_host:=0.0.0.0 port:=7447
```

다음으로 **Terminal 2 — Jetson Humble**에서 전체 실물 stack과 UI를 실행한다.
`192.168.30.50`은 LAB의 실제 유선 IP로 바꾼다.
`127.0.0.1`이나 Docker bridge 주소를 넣으면 안 된다:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/rby1_ros2_ws/install/setup.bash"
export RBY1_SDK_PATH="$HOME/sdk/rby1-sdk"
export LD_LIBRARY_PATH="$RBY1_SDK_PATH/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_DOMAIN_ID=0
ros2 launch rby1_vslam physical.launch.py \
  lab_host:=192.168.30.50 \
  robot_address:=192.168.30.1:50051 \
  robot_model:=m robot_version:=1_3 use_scan:=false
```

카메라 serial을 지정하려면 `serial_no:=123456789012`를 붙인다. D405나 IMU 없는 D435를 대신 선택하지 않도록 모델을 `d435i`로 제한했다.

Jetson에서 `base → link_head_2` 동적 TF는 robot state publisher/관절 피드백이 제공한다.
`link_head_2 → d435_camera_center → d435_link`는 초기 장착값이므로 실제 측정이 필요하다.

```bash
# Jetson: 결과 확인
ros2 topic echo /rby1/vslam/bridge_status --once
ros2 topic hz /rby1/vslam/camera_odometry
ros2 topic echo /rby1/vslam/slam_odom --once
```

### 5. UI로 Mapping하고 Point와 cuVSLAM 지도 저장

`physical.launch.py`가 RViz2와 `RBY1 VSLAM Mapping / Waypoints` UI를 함께 띄운다.
`bridge_status`가 `connected=true`, `tracking_ok=true`인 것을 확인한 다음 진행한다.

1. 로봇 옆에서 하드웨어 EMO를 잡을 수 있게 준비한다.
2. UI에서 `Prepare Power + Servo`, `Control Manager ON`, `Stream ON`을 순서대로 누른다.
3. 기본 `0.02 m/s`, `0.08 rad/s`로 hold-to-move 버튼을 누르고 맵 영역을 천천히 돈다.
4. 가능한 경우 시작 지점 근처로 돌아와 loop closure를 만든다.
5. 원하는 위치마다 이름을 입력하고 `Save Current Pose`를 누른다.

Point는 `rby1_vslam/data/rby1_vslam_waypoints.yaml`에 즉시 저장된다.
`colcon build --symlink-install`로 빌드하면 소스 패키지의 `data` 폴더를 사용하고,
일반 설치에서는 설치된 패키지의 `share/rby1_vslam/data` 폴더를 사용한다.
폴더와 YAML 파일은 첫 저장 시 생성된다. UI 로그의 `Waypoint file:`에서 실제 경로를
확인할 수 있으며, `waypoints_file:=/절대경로/points.yaml`로 덮어쓸 수 있다.
기존 `$HOME/rby1_maps/rby1_vslam_waypoints.yaml`은 자동 이동하지 않는다. 기존 포인트를
사용하려면 UI를 종료한 상태에서 새 경로로 복사하거나 기존 경로를 인자로 지정한다.
cuVSLAM 지도와 Point 파일은 서로 다른 파일이다. 별도 offline mapping은 필요 없고,
온라인 mapping을 충분히 수행한 뒤 LAB 컨테이너의 다른 터미널에서 지도를 저장한다.

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=85
mkdir -p /workspaces/isaac_ros-dev/maps
ros2 run rby1_vslam map_tool save \
  --path /workspaces/isaac_ros-dev/maps/rby1_lab_01
```

`use_scan:=false`에서는 장애물 감지와 회피가 전혀 없다. Mapping 중 Point 이동을 바로
시험한다면 빈 공간의 가까운 Point만 선택하고 `Navigate to Selected Point`를 누른다.

### 6. 저장 지도 Localization 후 Point 이동

Terminal 1의 mapping launch를 종료하고 같은 LAB 컨테이너에서 저장 지도를 연다.

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=85
ros2 launch rby1_vslam localization.launch.py \
  map_path:=/workspaces/isaac_ros-dev/maps/rby1_lab_01 \
  bind_host:=0.0.0.0 port:=7447
```

Jetson의 `physical.launch.py`는 그대로 두어도 TCP가 재연결된다. 영상이 다시 들어오면 LAB
컨테이너의 다른 터미널에서 초기 카메라 pose 힌트로 localization을 요청한다. 지도 작성
시작점과 같은 카메라 위치·방향에서 시작했다면 다음처럼 0을 사용한다.

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=85
ros2 run rby1_vslam map_tool localize \
  --path /workspaces/isaac_ros-dev/maps/rby1_lab_01 \
  --x 0.0 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0
```

이 힌트는 `base`가 아니라 지도상의 `d435_link` pose다. 다른 위치에서 시작하면 그
카메라의 대략적인 위치와 방향을 입력한다. LAB에서 localization 성공 로그가 나오고
Jetson에서 `tracking_ok=true` 및 `vslam_map → base` TF를 확인한 뒤 UI에서 저장된 Point를
선택해 `Navigate to Selected Point`를 누른다. 버튼이 velocity gate를 활성화하고 새 Nav2
goal을 보낸다. 초기 위치를 모르는 완전한 global localization이 필요할 때만 LAB PC에
cuVGL을 추가하며, 오늘의 pose-hint 절차와 Jetson에는 필요하지 않다.

## 인터페이스

| 토픽 | 타입 | 위치 / 방향 |
|---|---|---|
| `/d435/d435/infra1/image_rect_raw`, `infra2/image_rect_raw` | `sensor_msgs/Image` | UPC → LAB, mono8 raw |
| `/d435/d435/infra1/camera_info`, `infra2/camera_info` | `sensor_msgs/CameraInfo` | UPC → LAB |
| `/d435/d435/imu` | `sensor_msgs/Imu` | UPC → LAB |
| `/tf_static` | `tf2_msgs/TFMessage` | 카메라 내부 extrinsic만 UPC → LAB |
| `/rby1/vslam/camera_odometry` | `nav_msgs/Odometry` | LAB → UPC, `d435_link`의 VIO 위치 |
| `/rby1/vslam/camera_slam_odometry` | `nav_msgs/Odometry` | LAB → UPC, `d435_link`의 SLAM 위치 |
| `/rby1/vslam/odom`, `/rby1/vslam/slam_odom` | `nav_msgs/Odometry` | UPC에서 `base` 위치로 변환 |
| `/rby1/vslam/bridge_status` | `std_msgs/String` | 각 PC의 로컬 브릿지 JSON 상태 |
| `/rby1/vslam/timing` | `std_msgs/String` | recorder 구독 중에만 발행되는 session/stamp 내부 경계 이벤트 |
| `/rby1/vslam/navigation_event` | `std_msgs/String` | 요청 goal 좌표·UUID·최종 action 상태 진단 이벤트 |
| `/rby1/vslam/nav2_cmd_vel` | `geometry_msgs/Twist` | UPC Nav2 → cmd_raw 게이트 |
| `/rby1/cmd_raw` | `geometry_msgs/Twist` | UPC 출력, 기존 control 입력 |
| `/scan` | `sensor_msgs/LaserScan` | UPC Nav2 장애물 costmap 입력 |

Nav2 목표 action은 `/rby1/vslam/nav2/navigate_to_pose`와 `/rby1/vslam/nav2/navigate_through_poses`이다. 게이트 활성화는 `/rby1/vslam/enable` (`std_srvs/SetBool`), 정지·goal 취소는 `/rby1/vslam/cancel` (`std_srvs/Trigger`)이다.

기존 `/rby1/odom`, `odom → base` TF authority는 유지한다. LAB의 Isaac cuVSLAM은
LAB 로컬 RViz용 `vslam_map → vslam_odom → d435_link` TF를 발행하지만, 이 동적 TF는
TCP로 UPC에 전달하지 않는다. UPC에서는 navigation launch만 `vslam_map → odom`
보정 TF를 발행한다. 기존 AMCL/SLAM처럼 `odom`에 다른 parent TF를 발행하는 노드는
함께 실행하지 않는다.

640×480 mono8 스테레오 30 Hz의 영상 원본은 약 147 Mbps이다. 유선 1 Gbps부터 시작한다. TCP는 영상/보정/IMU 및 추정 결과용이고 로봇 전원이나 속도 명령을 네트워크 반대편에서 받아 실행하지 않는다. 신뢰할 수 있는 실험실 LAN/VPN에서만 포트를 노출한다. 자세한 큐/재연결 동작은 코드와 [실험 가이드](docs/experiment.md)를 참고한다.
