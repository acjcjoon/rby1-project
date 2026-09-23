# rby1_vslam

UPC의 Ubuntu 22.04 / ROS 2 Humble에서 D435i를 읽고, LAB PC의 Ubuntu 24.04 / ROS 2 Jazzy / Isaac ROS 4.5로 TCP 전송해 VSLAM을 계산한다. 결과를 UPC로 돌려받아 로봇 베이스 위치로 변환하고, **UPC의 Nav2**에서 경로계획·장애물 회피를 실행한 뒤 기존 **`/rby1/cmd_raw` (`geometry_msgs/Twist`)**에 발행한다.

이 패키지만 양쪽에 복사해 빌드한다. UPC에는 CUDA, Isaac ROS, Jazzy가 필요 없다. LAB에는 RealSense USB 드라이버가 필요 없다. 기존 Humble 시뮬레이션 컨테이너는 그대로 둔다.

**Nav2와 지도:** `navigation.launch.py`는 Navfn 경로계획 + holonomic DWB 제어 + velocity smoother + `/scan` 장애물 costmap을 실행한다. cuVSLAM 특징점 지도와 Nav2의 occupancy map은 다른 데이터다. 기본 Nav2는 VSLAM 좌표계에서 실시간 LiDAR로 만드는 20 m rolling costmap을 사용한다. 관측된 주행 가능한 영역 안에서 목표를 지정한다. 같은 `vslam_map` 좌표로 정렬된 occupancy map이 있다면 `occupancy_map:=/절대경로/map.yaml` 옵션으로 map_server/static layer를 사용한다.

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
  Nav2 (별도 실행, /scan + /rby1/odom 사용)
       │
  nav2_gate (기본 비활성, 입력 유효성 확인)
       │
  /rby1/cmd_raw → 기존 rby1_control → /rby1/cmd_vel → 기존 rby1_driver
```

로봇 구동·전원·서보·stream control은 기존 `rby1_control`과 드라이버 담당이다. 이 패키지는 이들을 시작하거나 전원을 켜지 않는다. 다른 `cmd_raw` 발행기와 동시에 실행하지 않는다.

카메라가 **항상 같은 머리 자세여야 하는 것은 아니다.** cuVSLAM은 카메라에 고정된 `d435_link`를 추적하고, UPC는 촬영 시점의 실제 관절 TF로 `base` 위치를 계산한다. 따라서 정확한 관절 TF와 장착 보정이 있으면 머리·몸통 운동을 분리할 수 있다. 첫 검증 때만 자세를 고정해 전송/추적/TF 문제를 나눠 확인한다. TF가 없으면 임의의 고정 변환으로 대체하지 않는다.

## 파일과 실행 위치

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `d435i.launch.py` | UPC | D435i 1대만 실행, 기존 `/d435/d435` 이름 |
| `upc.launch.py` | UPC | 카메라 + 장착 TF + TCP client + pose adapter |
| `lab.launch.py` | LAB Jazzy | TCP server + cuVSLAM, `mode` 선택 |
| `mapping.launch.py` | LAB Jazzy | 새 특징점 지도 작성 |
| `localization.launch.py` | LAB Jazzy | 기존 지도 선택, 이후 위치 힌트로 localize |
| `navigation.launch.py` | UPC | Nav2 + VSLAM 위치 TF + cmd_raw 게이트 |
| `map_tool` | LAB Jazzy | 지도 저장 / 힌트 기반 재위치 추정 |

`upc.launch.py`와 `d435i.launch.py`는 동시에 실행하지 않는다. 별도로 카메라를 켰다면 UPC launch에 `start_camera:=false`를 준다. mapping/localization/lab 중 하나만 LAB에서 실행한다.

## 먼저 설치

LAB 5080 / 드라이버 580 환경 구성은 **[LAB 설치 가이드](docs/lab_setup.md)**, 로봇·TF·지도·목표 이동 전체 절차는 **[실험 가이드](docs/experiment.md)**를 따른다.

UPC에서 기존 Humble 환경을 source한 뒤, `rby1_vslam` 폴더를 `~/rby1_vslam_ws/src/rby1_vslam`에 복사한다. 기존 다른 패키지를 이 workspace로 옮길 필요는 없다.

```bash
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-realsense2-camera ros-humble-realsense2-description \
  ros-humble-navigation2 ros-humble-nav2-bringup
cd ~/rby1_vslam_ws
rosdep install --from-paths src/rby1_vslam --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-select rby1_vslam
source install/setup.bash
```

기존 RealSense 드라이버가 설치되어 있으면 중복 설치하지 않고 먼저 버전/실행 프로파일을 확인한다. 여기의 UPC apt 경로는 설치가 간단한 Humble 경로이며, NVIDIA가 시험한 전체 RealSense 버전 조합과 같음을 보장하지 않는다. [버전 확인과 권장 조합](docs/experiment.md#카메라-확인)을 참고한다.

## 첫 실행

LAB 설치 완료 후 **LAB 컨테이너/Jazzy 터미널**:

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=85
ros2 launch rby1_vslam mapping.launch.py
```

**UPC Humble 터미널** — `192.168.30.50`은 LAB의 실제 유선 IP로 바꾼다. `ROS_DOMAIN_ID`는 기존 로봇 드라이버·TF·control과 같게 한다:

```bash
source /opt/ros/humble/setup.bash
source ~/rby1_vslam_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch rby1_vslam upc.launch.py lab_host:=192.168.30.50
```

카메라 serial을 지정하려면 `serial_no:=123456789012`를 붙인다. D405나 IMU 없는 D435를 대신 선택하지 않도록 모델을 `d435i`로 제한했다.

UPC에서 `base → link_head_2` 동적 TF는 기존 robot state publisher/관절 피드백이 제공해야 한다. 이 launch가 만드는 `link_head_2 → d435_camera_center → d435_link`는 기존 저장소 값을 복사한 초기 장착값이다. 실제 장착 측정이 필요하다. 기존 mount TF가 이미 있으면 `publish_mount_tf:=false`를 붙인다.

```bash
# UPC: 결과 확인. TF가 없어도 camera_odometry까지는 확인 가능하다.
ros2 topic echo /rby1/vslam/bridge_status
ros2 topic hz /rby1/vslam/camera_odometry
ros2 topic echo /rby1/vslam/slam_odom --once
```

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
| `/rby1/vslam/nav2_cmd_vel` | `geometry_msgs/Twist` | UPC Nav2 → cmd_raw 게이트 |
| `/rby1/cmd_raw` | `geometry_msgs/Twist` | UPC 출력, 기존 control 입력 |
| `/scan` | `sensor_msgs/LaserScan` | UPC Nav2 장애물 costmap 입력 |

Nav2 목표 action은 `/rby1/vslam/nav2/navigate_to_pose`와 `/rby1/vslam/nav2/navigate_through_poses`이다. 게이트 활성화는 `/rby1/vslam/enable` (`std_srvs/SetBool`), 정지·goal 취소는 `/rby1/vslam/cancel` (`std_srvs/Trigger`)이다.

기존 `/rby1/odom`, `odom → base` TF authority는 유지한다. Isaac 쪽 TF 자동 발행은 껐다. UPC의 navigation launch만 `vslam_map → odom` 보정 TF를 발행한다. 기존 AMCL/SLAM처럼 `odom`에 다른 parent TF를 발행하는 노드는 함께 실행하지 않는다.

640×480 mono8 스테레오 30 Hz의 영상 원본은 약 147 Mbps이다. 유선 1 Gbps부터 시작한다. TCP는 영상/보정/IMU 및 추정 결과용이고 로봇 전원이나 속도 명령을 네트워크 반대편에서 받아 실행하지 않는다. 신뢰할 수 있는 실험실 LAN/VPN에서만 포트를 노출한다. 자세한 큐/재연결 동작은 코드와 [실험 가이드](docs/experiment.md)를 참고한다.
