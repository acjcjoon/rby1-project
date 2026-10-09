# RBY1 VSLAM + Nav2 실행

UPC Jetson의 ROS 2 Humble에서 D435i·로봇 드라이버·Nav2·운영 UI를 실행한다.
LAB PC의 Isaac ROS 4.5/Jazzy에서 cuVSLAM을 실행한다. 영상과 IMU는 TCP 7447로
보내고, 추정 위치와 추적 상태를 UPC로 돌려받는다. Jetson에는 Isaac ROS를 설치하지 않는다.

설치는 [Jetson 실행 가이드](docs/jetson_physical.md)와
[LAB 설치 가이드](docs/lab_setup.md)를 따른다.

## 시작

UPC와 LAB에 같은 버전의 `rby1_vslam`을 빌드한다.

UPC:

```bash
source /opt/ros/humble/setup.bash
cd ~/rby1_ros2_ws
colcon build --symlink-install --packages-up-to \
  rby1_driver rby1_description rby1_control rby1_vslam
```

LAB의 Isaac ROS 컨테이너:

```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select rby1_vslam
source install/setup.bash
export ROS_DOMAIN_ID=85
ros2 launch rby1_vslam lab.launch.py mode:=mapping
```

LAB이 켜진 뒤 UPC 데스크톱의 저장소 루트에서 실행한다.
`<LAB-IP>`는 UPC에서 접근 가능한 LAB의 Wi-Fi IP로 바꾼다.

```bash
cd ~/rby1_ros2_ws/src/rby1-project
bash run_vslam_upc.sh lab_host:=<LAB-IP>
```

이 런처는 드라이버·로봇 TF·제어·D435i·TCP·위치 변환·Nav2·Qt·RViz를 함께 실행한다.
기본 workspace는 `~/rby1_ros2_ws`, UPC domain은 현재 환경 또는 `0`이다.
`RBY1_WORKSPACE_SETUP`, `RBY1_ROS_SETUP`으로 설치 경로를 바꿀 수 있다.
카메라는 `~/librealsense-rsusb-2.58.4/build-rsusb/Release`의 RSUSB 라이브러리를 사용한다.
다른 경로는 `RBY1_RSUSB_LIB_DIR`에 지정한다.

전원·서보·control manager·stream은 UI에서 켠다. 수동 버튼으로 이동하며 위치를 저장한 뒤,
저장된 Point를 선택하고 `Navigate to Selected Point`를 누르면 Nav2가 목표를 받는다.
Nav2 속도 게이트는 시작 시 OFF이며, 추적·위치·wheel odometry 상태가 유효해야 활성화된다.
기본 `use_scan:=false`에는 장애물 감지·회피가 없다. LiDAR가 준비되면 `use_scan:=true`를 지정한다.

## 지도와 Point

Point는 `vslam_map` 좌표의 이름·x·y·yaw를 저장한다. 기본 경로는
`data/rby1_vslam_waypoints.yaml`이며 UI의 `Waypoint file:` 로그에서 실제 경로를 확인한다.
`waypoints_file:=/절대경로/points.yaml`로 지정할 수도 있다.

cuVSLAM 특징점 지도는 LAB에서 저장한다.

```bash
mkdir -p /workspaces/isaac_ros-dev/maps
ros2 run rby1_vslam map_tool save \
  --path /workspaces/isaac_ros-dev/maps/rby1_lab_01
```

같은 mapping 세션에서는 저장한 Point로 바로 이동할 수 있다. 다음 실행에서도 그 Point를
쓰려면 같은 지도를 로드하고 위치를 다시 찾는다. LAB mapping을 종료한 뒤:

```bash
ros2 launch rby1_vslam lab.launch.py mode:=localization \
  map_path:=/workspaces/isaac_ros-dev/maps/rby1_lab_01
```

UPC를 다시 실행해 영상이 들어오면 LAB의 다른 터미널에서 지도 좌표의 초기 카메라 위치를 준다.
다음 0 값은 지도 작성 시작 위치와 같은 카메라 자세에서 시작하는 예시다.

```bash
ros2 run rby1_vslam map_tool localize \
  --path /workspaces/isaac_ros-dev/maps/rby1_lab_01 \
  --x 0.0 --y 0.0 --z 0.0 --roll 0.0 --pitch 0.0 --yaw 0.0
```

cuVSLAM 특징점 지도와 Nav2 occupancy map은 별개다. `vslam_map`에 정렬된 occupancy
map이 있으면 UPC 실행에 `occupancy_map:=/절대경로/map.yaml`을 추가한다.

## 설정

| 파일 | 용도 |
|---|---|
| `config/bridge.yaml` | TCP·스테레오 큐 설정 |
| `config/d435i.yaml` | 카메라 프로파일·노출·IMU |
| `config/isaac_vslam.yaml` | LAB cuVSLAM 설정 |
| `config/navigation.yaml` | Theta*·MPPI·costmap·속도 게이트 |

TCP 송수신은 독립 스레드이며, 영상은 원래 촬영 timestamp를 유지한 스테레오 쌍으로 전달한다.
수신 시 ROS executor를 즉시 깨워 발행한다. 기본 영상 큐는 송신·수신 각각 3쌍이며,
반환 pose는 종류별 32개를 순서대로 처리한다. 큐가 차거나 영상이 오래되면 폐기 수를
`bridge_status`에 기록한다. TCP 송신 버퍼는 운영체제의 자동 조절을 사용한다.

영상 큐와 버퍼는 `bridge.yaml`의 `stereo_tx_queue_size`, `stereo_rx_queue_size`,
`socket_send_buffer_bytes`에서 설정한다. 별도 파일은 UPC와 LAB 각각
`bridge_params_file:=/절대경로/bridge.yaml`로 지정한다. 큐는 일시적인 Wi-Fi 지연을
흡수하며, 지속적인 대역폭 부족에서는 프레임 손실이 남을 수 있다.

IMU는 기본 ON이다. 끌 경우 UPC와 LAB 실행에 모두 `enable_imu:=false`를 지정한다.
두 PC의 시계는 chrony/NTP로 맞춘다. ROS domain은 달라도 TCP 통신이 가능하다.

## 위치와 TF

cuVSLAM은 `d435_link`를 추적한다. `pose_adapter`가 촬영 시점의 관절 TF로 `base` 위치를
계산하고, `localization_tf`가 `vslam_map → odom` 보정을 발행한다.
기존 드라이버가 `odom → base_footprint`를 발행하며, Nav2는 wheel odometry와 전역 보정을 사용한다.
다른 위치 추정 노드가 같은 TF를 함께 발행하지 않도록 한다.

기본 장착 TF는 다음과 같다. 실측값은 `mount_x/y/z/roll/pitch/yaw`로 지정한다.
각도 단위는 radian이다. 기존 장착 TF 발행기가 있으면 `publish_mount_tf:=false`를 사용한다.

```text
link_head_2 → d435_camera_center: xyz=(0.0464, 0, 0.066), pitch=-2°
d435_camera_center → d435_link: xyz=(0.02185, 0.0175, 0)
```

UPC에서 상태를 확인한다.

```bash
source ~/rby1_ros2_ws/install/setup.bash
ros2 topic echo /rby1/vslam/bridge_status --once
ros2 topic echo /rby1/vslam/navigation_status --once
ros2 run tf2_ros tf2_echo vslam_map base
```

`connected`와 `tracking_ok`가 true여야 한다. camera odometry만 있고 base 위치가 없으면
`base → d435_link` TF와 joint states를 확인한다. 연결되지 않으면 LAB 프로세스·Wi-Fi IP·
TCP 7447·컨테이너 네트워크를 확인한다.
