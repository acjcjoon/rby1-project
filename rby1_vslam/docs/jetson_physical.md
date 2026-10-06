# RBY1 실물 VSLAM: Jetson ↔ LAB PC

Jetson은 Isaac ROS를 설치하거나 실행하지 않는다.

```text
RBY1 Jetson / ROS 2 Humble
  D435i + robot driver + rby1_control + Nav2 + RViz/UI
                  │
                  │ TCP 7447 (stereo/IMU →, VSLAM pose ←)
                  ▼
LAB PC / ROS 2 Jazzy / Isaac ROS 4.5
  bridge server + cuVSLAM + map save/localize
```

두 PC의 ROS 배포판과 `ROS_DOMAIN_ID`는 달라도 된다. 데이터는 DDS가 아니라 TCP
bridge로 전달된다. TCP 포트는 하나지만 session 내부의 송신과 수신은 독립 worker가
담당하고, 반환 pose burst는 bounded FIFO로 보존한다.

## 0. 전제와 안전

- 실물 하드웨어 EMO를 즉시 누를 수 있는 사람이 로봇 옆에 있어야 한다.
- 처음에는 로봇 주위를 비우고 `0.02 m/s`, `0.08 rad/s`로 시험한다.
- `use_scan:=false`는 장애물 감지·회피를 제공하지 않는다.
- UI의 빨간 정지는 소프트웨어 정지다. 하드웨어 EMO를 대체하지 않는다.
- 실물 launch는 collision 상태를 무시하지 않으며 전원·서보·stream도 자동으로 켜지 않는다.

## 1. LAB PC 유선 IP 확인

LAB PC 호스트에서 실행한다.

```bash
hostname -I
ip -4 -br addr
```

Jetson과 같은 유선망의 주소를 고른다. 아래 예시는 `192.168.30.50`이다.
`127.0.0.1`, `172.17.x.x` 같은 Docker bridge 주소, 다른 망의 Wi-Fi 주소를 쓰지 않는다.

Jetson에서 도달 가능한지 확인한다.

```bash
ping -c 3 192.168.30.50
```

LAB 방화벽을 사용한다면 Jetson 주소에서 들어오는 TCP 7447만 허용한다. Isaac ROS
컨테이너는 host network를 사용하는 구성이 가장 단순하다.

## 2. Jetson 빌드

사용자명이 `nvidia-tegra`인지 확실하지 않아도 된다. `$HOME`을 사용한다.

```bash
whoami
echo "$HOME"

export RBY1_WS="$HOME/rby1_ros2_ws"
export RBY1_SDK_PATH="$HOME/sdk/rby1-sdk"  # 실제 SDK 위치로 수정
export LD_LIBRARY_PATH="$RBY1_SDK_PATH/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$RBY1_WS"
source /opt/ros/humble/setup.bash

test -d "$RBY1_WS/src/rby1-project/rby1_vslam"
test -d "$RBY1_WS/src/rby1-ros2/rby1_driver"
```

필요한 Humble 패키지가 없다면 한 번만 설치한다.

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions python3-rosdep python3-pyqt5 \
  ros-humble-realsense2-camera ros-humble-realsense2-description \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-robot-state-publisher ros-humble-tf2-ros
```

워크스페이스를 빌드한다.

```bash
cd "$RBY1_WS"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-up-to \
  rby1_driver rby1_description rby1_control rby1_vslam
source install/setup.bash

ros2 pkg prefix rby1_vslam
ros2 launch rby1_vslam physical.launch.py --show-args
```

## 3. LAB PC 빌드

LAB 호스트의 Isaac ROS workspace에 같은 버전의 `rby1_vslam` 폴더를 둔다. 예시는
`$HOME/workspaces/isaac_ros-dev/src/rby1_vslam`이다.

호스트에서 먼저 workspace 변수를 설정해야 `isaac-ros activate`가 실패하지 않는다.

```bash
export ISAAC_ROS_WS="$HOME/workspaces/isaac_ros-dev"
test -d "$ISAAC_ROS_WS/src/rby1_vslam"
isaac-ros activate
```

LAB의 Isaac ROS Jazzy 컨테이너 안에서 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select rby1_vslam
source install/setup.bash
ros2 pkg prefix isaac_ros_visual_slam
```

## 4. Mapping 실행

### Terminal 1 — LAB PC Isaac ROS 컨테이너

```bash
export ISAAC_ROS_WS="$HOME/workspaces/isaac_ros-dev"
isaac-ros activate

source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=85

ros2 launch rby1_vslam mapping.launch.py \
  bind_host:=0.0.0.0 port:=7447
```

LAB 로그에서 bridge가 연결을 기다리는 상태는 정상이다. Jetson이 연결되면 영상과
cuVSLAM tracking이 시작된다.

### Terminal 2 — RBY1 Jetson

```bash
export RBY1_WS="$HOME/rby1_ros2_ws"
export RBY1_SDK_PATH="$HOME/sdk/rby1-sdk"
export LD_LIBRARY_PATH="$RBY1_SDK_PATH/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_DOMAIN_ID=0

cd "$RBY1_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rby1_vslam physical.launch.py \
  lab_host:=192.168.30.50 \
  port:=7447 \
  robot_address:=192.168.30.1:50051 \
  robot_model:=m \
  robot_version:=1_3 \
  use_scan:=false
```

실제 URDF가 M 1.2이면 `robot_version:=1_2`로 바꾼다. D435i가 여러 대면
`serial_no:=숫자시리얼`을 추가한다. 기존 driver/control/camera를 따로 실행했다면
`start_driver:=false`, `start_control:=false`, `start_camera:=false`로 중복을 막는다.

`physical.launch.py`가 실행하는 구성:

- RBY1 driver, robot_state_publisher, `base_footprint → base` TF
- `rby1_control`과 실물 collision 차단
- D435i, 장착 TF, Jetson TCP client, pose adapter
- LiDAR 없는 Nav2와 기본 OFF 상태의 velocity gate
- RViz2와 waypoint operator UI

### Terminal 3 — Jetson 상태 확인

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source "$HOME/rby1_ros2_ws/install/setup.bash"

ros2 topic hz /d435/d435/infra1/image_rect_raw
ros2 topic echo /rby1/vslam/bridge_status --once
ros2 topic hz /rby1/vslam/camera_odometry
ros2 run tf2_ros tf2_echo vslam_map base
```

`bridge_status`가 `connected: true`, `tracking_ok: true`가 된 뒤 움직인다.
`base → link_head_2 → d435_link` TF가 없으면 base pose가 나오지 않는다. 기본 mount 값은
초기값이므로 실제 장착을 측정해 `mount_x/y/z/roll/pitch/yaw`로 보정한다.

## 5. UI로 Mapping과 Point 저장

1. 하드웨어 EMO와 작업 공간을 확인한다.
2. UI에서 `Prepare Power + Servo`를 누른다.
3. `Control Manager ON`, `Stream ON`을 누른다.
4. VSLAM이 `tracking OK`인지 확인한다.
5. hold-to-move 버튼으로 천천히 이동한다. 버튼을 놓으면 0 명령이 전송된다.
6. 원하는 위치에서 이름을 확인하고 `Save Current Pose`를 누른다.

Point는 `rby1_vslam/data/rby1_vslam_waypoints.yaml`에 저장된다. `--symlink-install`
빌드에서는 소스 패키지, 일반 설치에서는 설치된 `share/rby1_vslam` 안의 `data`를
사용한다. 실제 경로는 UI 로그의 `Waypoint file:`에서 확인한다. cuVSLAM 지도 파일과는
별도이며, `waypoints_file:=/절대경로/points.yaml`로 경로를 지정할 수도 있다.
기존 `$HOME/rby1_maps/rby1_vslam_waypoints.yaml`은 자동 이동하지 않는다.

지도 작성은 온라인이다. 별도의 offline mapping 단계는 필수가 아니다. 충분히 이동하고
시작점 근처로 돌아와 loop closure를 만든 다음 LAB에서 지도를 저장한다.

### LAB PC — 지도 저장

```bash
mkdir -p /workspaces/isaac_ros-dev/maps
ros2 run rby1_vslam map_tool save \
  --path /workspaces/isaac_ros-dev/maps/rby1_lab_01
```

저장 경로는 LAB 컨테이너에서 보이는 새 경로여야 한다. 같은 mapping 세션에서는 저장한
Point를 선택하고 `Navigate to Selected Point`를 눌러 Nav2를 시험할 수 있다. 이 버튼이
velocity gate를 명시적으로 활성화한 뒤 새 goal을 보낸다. `use_scan:=false`에서는
장애물을 전혀 피하지 못하므로 빈 공간에서만 짧은 목표를 사용한다.

## 6. 저장 지도에서 다시 시작

LAB에서 mapping launch를 종료하고 localization launch를 실행한다.

```bash
ros2 launch rby1_vslam localization.launch.py \
  map_path:=/workspaces/isaac_ros-dev/maps/rby1_lab_01 \
  bind_host:=0.0.0.0 port:=7447
```

Jetson의 `physical.launch.py`를 다시 실행해 영상이 들어온 뒤 LAB의 다른 터미널에서
초기 위치 힌트를 보낸다. 지도 작성 시작 위치와 같은 카메라 자세이면 0 값을 쓴다.

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=85

ros2 run rby1_vslam map_tool localize \
  --path /workspaces/isaac_ros-dev/maps/rby1_lab_01 \
  --x 0.0 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0
```

다른 곳에서 시작하면 지도 좌표계의 대략적인 `d435_link` pose를 넣는다.
`localized_in_exist_map=Yes`, Jetson의 `tracking_ok=true`, `vslam_map → base` TF를
확인한 뒤 저장한 Point로 이동한다.

cuVGL은 이 기본 절차에 필요하지 않다. 시작 pose를 모르는 임의 위치에서 자동으로 찾고
싶을 때만 LAB PC에 cuVGL을 추가한다. Jetson에는 설치하지 않는다.

## 7. 자주 생기는 문제

- `Connection refused`: LAB launch가 먼저 떠 있는지, LAB 유선 IP, TCP 7447,
  컨테이너 host network를 확인한다.
- `127.0.0.1`로 연결됨: Jetson 자신이다. LAB PC 유선 IP로 바꾼다.
- camera odometry만 있고 base pose가 없음: `base → d435_link` TF와 joint states를 본다.
- UI는 떴지만 이동 안 함: collision/EMO, power/servo, control manager, stream을 확인한다.
- Nav2가 움직이지 않음: bridge/tracking/localization/wheel odom/gate 상태를 확인한다.
- 기존 프로세스와 충돌: driver, camera, `/rby1/cmd_raw` publisher 중복을 확인한다.
