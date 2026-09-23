# UPC ↔ LAB 실험 순서

## 환경과 책임

| 구성 | 담당 |
|---|---|
| UPC Humble | D435i USB/센서 timestamp, TCP client, 실제 관절 TF, 카메라→베이스 pose 변환 |
| LAB Jazzy | TCP server, RTX 5080 cuVSLAM, 특징점 지도 저장·재위치 추정 |
| UPC Nav2 | wheel odom + VSLAM 위치 보정 + LiDAR로 경로계획·로컬 회피 |
| UPC cmd_raw 게이트 | Nav2 출력의 신선도·연결·추적·좌표 보정 확인 후 `/rby1/cmd_raw` 발행 |
| 기존 rby1_control/driver | 최종 속도 제한/상태 판단, `/rby1/cmd_vel`, 전원·서보·stream |

UPC 터미널은 기존 robot workspace를 먼저 source하고 이 패키지 workspace를 마지막에 source한다. 기존 driver, robot state publisher, LiDAR, control과 **같은 ROS_DOMAIN_ID**를 사용한다. 예제의 0을 이미 다른 값으로 운영 중이라면 기존 값으로 바꾼다. LAB Jazzy만 85처럼 다른 domain을 사용한다.

`run_planner_ui.sh`는 두 카메라와 기존 navigation/명령 발행기를 함께 띄우므로 이 실험의 시작 명령으로 사용하지 않는다. 기존 로봇 driver + state publisher + control을 필요한 만큼 별도로 실행한다. 이 패키지는 다른 패키지 파일을 수정하거나 해당 launch를 자동으로 포함하지 않는다.

## 카메라 확인

이 패키지는 **D435i 한 대**를 사용한다. `/d435/d435` namespace는 기존 저장소 이름과 같지만 IMU 없는 D435는 선택하지 않는다. 연결된 모델과 serial을 RealSense Viewer 또는 `rs-enumerate-devices`로 확인한다.

NVIDIA 4.5 공식 RealSense 시험 조합은 firmware **5.16.0.1**, librealsense **2.56.3**, realsense-ros **r/4.56.3**이다. UPC는 Humble이므로 이 전체 조합을 이 패키지가 자동 설치하거나 공식 인증한다고 보장하지 않는다. 현재 UPC의 Humble apt 드라이버로 먼저 아래 스트림/TF를 확인하고, 문제가 있으면 [공식 버전 정보](https://nvidia-isaac-ros.github.io/v/release-4.5/getting_started/sensors/realsense_setup.html)와 [RealSense ROS 소스](https://github.com/realsenseai/realsense-ros)를 기준으로 별도 sensor workspace에서 맞춘다. LAB 쪽 RealSense 설치 스크립트와 CUDA를 UPC에 그대로 적용하지 않는다.

기본 `config/d435i.yaml`은 다음과 같다:

- 좌/우 IR rectified mono8, `640×480 @ 30 Hz`, raw 전송
- IR emitter OFF, 좌우 frame 동기화 ON
- gyro 200 Hz + accel 250 Hz, linear interpolation 통합 IMU
- color/depth OFF, 내부 static TF ON

카메라만 먼저 확인할 때:

```bash
ros2 launch rby1_vslam d435i.launch.py serial_no:=123456789012
```

다른 UPC 터미널에서:

```bash
ros2 topic hz /d435/d435/infra1/image_rect_raw
ros2 topic hz /d435/d435/infra2/image_rect_raw
ros2 topic hz /d435/d435/imu
ros2 topic echo /d435/d435/infra1/camera_info --once
ros2 run tf2_ros tf2_echo d435_link d435_infra1_optical_frame
ros2 run tf2_ros tf2_echo d435_link d435_gyro_optical_frame
```

IR 둘 다 약 30 Hz, IMU 약 200 Hz인지 확인한다. 영상과 해당 `CameraInfo`의 stamp가 같아야 브릿지가 stereo bundle을 만들 수 있다. `d435_link`에서 좌우 optical/IMU optical frame으로 이어지는 실제 제조사 보정 TF를 사용한다. 가짜 baseline TF를 작성하지 않는다.

통합 UPC launch로 넘어가기 전에 위 카메라 launch를 종료한다. 계속 켜 둘 경우 아래 UPC launch에 `start_camera:=false`를 추가한다.

## TF와 기존 구동 준비

UPC에서 확인:

```bash
ros2 topic echo /rby1/odom --once
ros2 topic hz /rby1/joint_states
ros2 run tf2_ros tf2_echo base link_head_2
ros2 run tf2_ros tf2_echo odom base
```

`base → link_head_2`는 실제 관절 피드백에 따라 갱신되어야 한다. `odom → base`는 기존 wheel odometry 담당 노드가 발행한다. 실제 wheel odom frame/child가 `odom`/`base`인지 반드시 메시지로 확인한다. 예를 들어 실제 child가 `base_footprint`이고 이를 기준으로 쓸 경우 **UPC launch와 navigation launch 모두 `base_frame:=base_footprint`**로 실행한다. 그 frame에서 head까지 실제 TF 체인이 있어야 한다. 실제 odom frame 이름이 다르면 navigation의 `odom_frame:=실제이름`도 맞춘다.

UPC launch는 기본으로 다음 장착 TF 두 개만 추가한다:

```text
link_head_2 → d435_camera_center: xyz=(0.0464, 0, 0.066) m, pitch=-2°
d435_camera_center → d435_link: xyz=(0.02185, 0.0175, 0) m, rotation=0
```

이는 기존 `rby1_camera/config/camera_system.yaml`의 초기 장착값이다. 실제 D435i 설치 위치를 측정해서 조정한다. `mount_x/y/z/roll/pitch/yaw` launch 인자로 첫 변환을 바꿀 수 있으며 회전 단위는 rad이다. 두 번째 CAD offset은 `upc.launch.py`에 명시되어 있다. 기존 장착 TF가 실행 중이면 `publish_mount_tf:=false`로 중복을 막는다.

카메라 위치를 `base`로 변환하는 식은 `T_world_base(t) = T_world_camera(t) × T_camera_base(t)`이다. 촬영 시점 TF를 조회하므로 머리를 움직여도 원리상 가능하다. 초기에는 머리/몸통을 고정하고, 이후 제자리 머리 운동에서 베이스 위치가 안정적인지 검증한다. `/rby1/vslam/*odom`의 twist는 카메라 속도를 베이스 속도로 단순 복사하지 않으며 큰 covariance로 표시한다. Nav2 속도 피드백은 기존 `/rby1/odom`을 사용한다.

기존 control이 필요하면 준비된 robot workspace에서 별도 실행한다:

```bash
ros2 launch rby1_control control.launch.py \
  robot_address:=192.168.30.1:50051 robot_model:=m
```

driver와 state publisher의 로봇 URDF/parameter 경로는 기존 LAB/UPC 설정을 사용한다. 전원·서보·control stream은 기존 절차로 준비한다. 이 패키지는 자동으로 이를 변경하지 않는다.

## 1. Mapping

LAB 컨테이너/Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=85
ros2 launch rby1_vslam mapping.launch.py
```

UPC/Humble (`192.168.30.50`은 실제 LAB IP):

```bash
source /opt/ros/humble/setup.bash
# 기존 robot workspace setup.bash를 여기서 source
source ~/rby1_vslam_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch rby1_vslam upc.launch.py lab_host:=192.168.30.50
```

LAB에서 `/d435/d435/infra1/image_rect_raw`와 `/visual_slam/status`가 들어오고, UPC에서 아래 결과가 나와야 한다:

```bash
ros2 topic echo /rby1/vslam/bridge_status
ros2 topic hz /rby1/vslam/camera_odometry
ros2 topic hz /rby1/vslam/slam_odom
```

처음 수 초간 카메라를 정지해 IMU 초기화 시간을 준다. 지도 작성은 기존 수동 조작 수단으로 천천히 이동하며 수행한다. 이 단계에서 navigation launch는 실행하지 않는다. 기능을 검증한 뒤 원래 장소로 돌아와 loop closure와 위치 보정을 확인한다.

LAB의 새 Jazzy 터미널에서 지도 저장:

```bash
mkdir -p /workspaces/isaac_ros-dev/maps
ros2 run rby1_vslam map_tool save \
  --path /workspaces/isaac_ros-dev/maps/lab_trial_01
```

cuVSLAM `.mdb` 특징점 지도가 LAB workspace에 저장된다. `map_tool`의 완료 메시지와 파일을 확인한다. 지도는 UPC에 복사할 필요가 없다. map_tool은 4.5 비동기 API의 서비스 수락과 실제 완료를 구분하기 위해 `/rosout`의 해당 component 로그를 기다린다. `visual_slam`의 INFO 로그/rosout을 끄지 않는다. timeout은 server-side 작업을 취소하지 않으므로 반복 요청 전에 완료/실패 상태를 확인한다.

## 2. Localization

UPC의 camera/bridge는 유지한다. Nav2가 실행 중이면 먼저 아래 명령으로 gate를 비활성화하고 현재 goal 취소를 확인한 뒤 navigation launch를 종료한다. 그 다음 LAB mapping launch를 종료한다:

```bash
ros2 service call /rby1/vslam/enable std_srvs/srv/SetBool '{data: false}'
```

LAB:

```bash
ros2 launch rby1_vslam localization.launch.py \
  map_path:=/workspaces/isaac_ros-dev/maps/lab_trial_01
```

영상과 IMU가 들어온 후 LAB 새 터미널에서:

```bash
ros2 run rby1_vslam map_tool localize \
  --path /workspaces/isaac_ros-dev/maps/lab_trial_01 \
  --x 0.0 --y 0.0 --z 0.0 --roll 0.0 --pitch 0.0 --yaw 0.0
```

위 값은 **지도상의 `d435_link` 위치/자세 힌트**이며 `base` 위치가 아니다. 지도 작성 시작 지점과 같은 카메라 위치·방향에서 identity hint를 사용한다. 다른 위치에서 시작하면 실제 map 좌표의 대략적인 camera pose를 넣는다. 위치는 m, 회전은 rad이다. cuVSLAM은 무힌트 전역 위치찾기를 하지 않는다.

이 launch는 자동 identity localization을 켜지 않는다. 위 명령이 성공 완료를 확인하고, LAB diagnostics `localized_in_exist_map`이 `Yes`가 되어야 브릿지가 localization 모드의 `tracking_ok`를 허용한다. 최초 재위치 추정 전의 새 local map 원점을 기존 지도 좌표로 잘못 사용해 Nav2가 출발하지 않도록 했다. 이미 localization에 성공한 이후에는 이 flag가 재요청 중에도 `Yes`일 수 있으므로 **localize/reset/map 변경 전에 항상 gate를 비활성화**한다. 완료 확인 후 TF를 검사하고 다시 활성화한다.

`enable_localization_n_mapping=true`는 지도 API 사용에 필요하다. 이 모드는 저장 지도를 이용해 재위치 추정을 하는 것이며 지도 업데이트를 완전히 금지하는 read-only localization 모드가 아니다. [cuVSLAM 지도 API와 frame 조건](https://nvidia-isaac-ros.github.io/v/release-4.5/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)

## 3. Nav2 → cmd_raw

**전제:** VSLAM/base pose 정상, wheel odom과 TF 정상, `/scan`과 그 TF 정상, 기존 control/driver 준비, 다른 `/rby1/cmd_raw` 발행기 종료. Nav2의 obstacle/inflation costmap은 LiDAR가 필요하다. D435i의 IR만으로 장애물 costmap이 생기는 것은 아니다.

기존 LiDAR launch를 별도로 사용하는 예:

```bash
ros2 launch rby1_bringup lidar.launch.py host_ip:=192.168.30.2
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base base_scan
```

`host_ip`는 LiDAR UDP를 받는 실제 UPC IP로 바꾼다. 기존 launch는 `base_footprint → base_scan` TF를 사용하므로 `base`와 `base_footprint` 사이 TF도 확인한다. 센서가 이미 켜져 있으면 중복 실행하지 않는다.

UPC:

```bash
sudo apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup
ros2 launch rby1_vslam navigation.launch.py
```

Nav2 노드들은 `/rby1/vslam/nav2` 아래 실행된다. lifecycle은 자동 활성화되지만 **cmd_raw 게이트는 기본 비활성**이다. 이 launch는 카메라/bridge를 재실행하지 않는다. `localization_tf`가 `vslam_map → odom`을 추가하며 기존 `odom → base`는 그대로 사용한다.

```bash
ros2 lifecycle get /rby1/vslam/nav2/controller_server
ros2 lifecycle get /rby1/vslam/nav2/bt_navigator
ros2 run tf2_ros tf2_echo vslam_map base
ros2 topic echo /rby1/vslam/localization_status
ros2 topic echo /rby1/vslam/navigation_status
ros2 topic info /rby1/cmd_raw --verbose
```

frame·토픽이 다르면 launch 인자 `base_frame`, `odom_frame`, `wheel_odom_topic`, `scan_topic`을 맞춘다. 다른 Nav2/AMCL/SLAM이 같은 odom parent TF를 발행하지 않아야 한다.

현재 위치와 costmap을 확인한 뒤 gate를 활성화한다:

```bash
ros2 service call /rby1/vslam/enable std_srvs/srv/SetBool '{data: true}'
```

응답이 `success: true`여야 한다. 현재 pose, wheel odom, bridge/VSLAM, localization TF 상태가 오래되거나 다른 raw command publisher가 있으면 거절한다. 게이트 시작 시 기존 Nav2 목표의 취소 응답과 종료 상태도 확인하므로 `cancel_pending`이면 완료를 기다린다. **활성화 성공 이후에 새 목표**를 보낸다.

Nav2 목표 예시 — **`x/y`는 상대 이동량이 아니라 `vslam_map` 절대 좌표**다. 먼저 `/rby1/vslam/slam_odom`의 현재 위치를 확인하고, 관측된 빈 공간의 가까운 목표로 수정한다:

```bash
ros2 action send_goal /rby1/vslam/nav2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: vslam_map}, pose: {position: {x: 0.3, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' \
  --feedback
```

RViz에서는 Fixed Frame=`vslam_map`, `/rby1/vslam/nav2/global_costmap/costmap`, `/rby1/vslam/nav2/local_costmap/costmap`, `/rby1/vslam/nav2/plan`, `/scan`, TF를 표시한다. Nav2 panel의 namespace를 `/rby1/vslam/nav2`로 맞춘 경우 GUI 목표도 사용할 수 있다. CLI action이 namespace 확인에 가장 직접적이다.

현재 설정은 holonomic DWB이며 최대 평면 속도 0.15 m/s, 회전 0.35 rad/s부터 시작한다. 실제 로봇 외곽/접은 팔까지 반영해 `config/navigation.yaml`의 `robot_radius` 또는 footprint와 inflation을 조정한다. 기본 0.45 m radius는 실측값이 아니다. 팔을 뻗은 상태의 충돌 형상을 반영하지 않는다.

정지 및 현재 Nav2 목표 취소:

```bash
ros2 service call /rby1/vslam/cancel std_srvs/srv/Trigger '{}'
ros2 service call /rby1/vslam/enable std_srvs/srv/SetBool '{data: false}'
```

게이트는 연결 끊김, 추적 실패, 오래된 센서 위치/TF, 위치 보정 급변 등을 감지하면 zero Twist를 내고 기존 goal을 취소한다. 이 경우 정상 복구해도 자동 재출발하지 않으며 명시적인 re-enable과 새 goal이 필요하다. Nav2 속도 명령만 끊긴 경우에는 정상 goal 완료 뒤의 silence일 수 있으므로 zero를 내되 armed 상태를 유지한다. LiDAR freshness는 Nav2 costmap의 `expected_update_rate`, controller 정지, velocity smoother/gate timeout으로 처리한다. 별도의 인증된 충돌 안전장치는 아니다. 기존 rby1_control의 cmd_raw watchdog도 유지한다.

## 지도/경로의 한계

cuVSLAM은 위치를 제공하고 Nav2는 costmap에 대해 경로를 계산한다. 기본 global costmap은 현재 위치 주변 20×20 m rolling window이며 local은 5×5 m이다. `/scan`으로 관측하지 않은 영역은 미지 영역이고 planner는 이를 통과하지 않는다. 따라서 저장한 visual map 전체를 알고 있다고 가정해 멀리 목표를 주면 경로가 실패할 수 있다.

건물 전체 장거리 경로가 필요하면 별도의 occupancy map 생성 절차와 그 map의 좌표를 `vslam_map`에 맞추는 과정이 필요하다. **이미 같은 좌표계로 정렬된 지도**가 있을 때 UPC에서:

```bash
ros2 launch rby1_vslam navigation.launch.py \
  occupancy_map:=/home/robot/maps/lab_aligned/map.yaml
```

`occupancy_map` 경로는 UPC 파일시스템의 실제 절대 경로로 바꾼다. 이 옵션은 Nav2 map_server를 시작하고 global costmap을 static/obstacle/inflation layer로 전환하며 rolling window를 끈다. local costmap은 계속 실시간 LiDAR를 쓴다. cuVSLAM `.mdb`를 이 인자에 넣으면 안 된다. 이 패키지는 특징점 지도를 occupancy map으로 변환하거나 서로 다른 지도 원점을 자동 정렬하지 않는다. [Humble Nav2 launch](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py), [Humble Nav2 기본 파라미터](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)

큰 loop closure나 재위치 추정으로 위치가 바뀌었으면, 게이트를 비활성화한 상태에서 완료된 pose/TF를 확인한 뒤 과거 위치에 남은 장애물 관측을 비운다:

```bash
ros2 service call /rby1/vslam/nav2/local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap '{}'
ros2 service call /rby1/vslam/nav2/global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap '{}'
```

LiDAR가 costmap을 다시 채운 것을 확인하고 re-enable 후 새 목표를 보낸다.

## 전송/지연과 문제 확인

| 증상 | 먼저 확인 |
|---|---|
| `connected=false` | LAB launch, host/port, 컨테이너 network, TCP 7447 방화벽 |
| 연결됐지만 LAB 영상이 없음 | D435i 토픽 이름, CameraInfo stamp, 좌우 pair stamp, 원본 이미지 age |
| `tracking_ok=false` | `/visual_slam/status`, mono8, optical/IMU TF, IMU 초기화, 특징/조명 |
| camera_odometry만 있고 base odom 없음 | `base↔d435_link` 촬영 시점 TF, UPC joint feedback, 시계 차이 |
| localization 후 gate 활성화 안 됨 | 실제 localize 완료, diagnostics의 saved-map localization 성공, wheel odom/TF |
| Nav2가 path를 못 찾음 | goal이 rolling window 안인지, LiDAR가 free space를 관측했는지, footprint/inflation |
| cmd_raw는 나오는데 로봇 정지 | 기존 control의 상태 gate, driver 전원·서보·stream 준비 |

image raw data는 binary로 보내고 메타데이터만 JSON으로 보낸다. 일반 ROS 메시지는 명시적인 schema로 복원하며 pickle/CDR 배포판 호환성에 기대지 않는다. stereo는 쌍 단위로 처리하고 누적 지연을 제한한다. 재연결 시 새 session과 calibration/static TF를 전달한다. TCP 재전송/혼잡으로 원본 timestamp가 오래되면 주행용 입력으로 사용하지 않는다.

유선 1 Gbps에서 먼저 확인한다. 기본 영상만 약 18.4 MB/s이며 UPC USB/CPU와 네트워크 부하도 관찰한다. `config/isaac_vslam.yaml`의 IMU noise는 NVIDIA 예제값이므로 정밀 성능 비교 전에 실제 IMU 보정을 진행한다.

## 기존 Humble 시뮬레이션/rosbag 재생

실기 UPC 대신 **Humble 시뮬레이션 컨테이너**에서 `upc.launch.py`를 실행할 수 있다. 단, 시뮬레이션 센서가 아래와 같은 interface를 제공해야 한다:

- `/d435/d435/infra{1,2}/image_rect_raw`: 보정된 mono8 stereo와 같은 stamp의 CameraInfo
- 실제 stereo baseline/optical orientation TF와 `base↔d435_link` 관절 TF
- VIO일 경우 같은 clock의 IMU; 없으면 양쪽 launch `enable_imu:=false`
- `/clock`의 단일 원천, Nav2용 wheel odom과 `/scan`

```bash
# Humble 시뮬레이션 컨테이너
ros2 launch rby1_vslam upc.launch.py lab_host:=192.168.30.50 \
  start_camera:=false publish_mount_tf:=false \
  use_sim_time:=true forward_clock:=true enable_imu:=false

# LAB Jazzy
ros2 launch rby1_vslam mapping.launch.py \
  use_sim_time:=true forward_clock:=true enable_imu:=false

# Humble, Nav2도 실험할 때
ros2 launch rby1_vslam navigation.launch.py use_sim_time:=true
```

시뮬레이션과 실기의 토픽이 자동으로 같아지는 것은 아니다. 이 패키지는 기존 시뮬레이터를 수정하지 않았으므로 센서 출력/TF를 위 interface로 맞춘 후 재사용한다. bag을 기록할 공간이 부족한 UPC 대신 LAB workspace에서 이미지/IMU/CameraInfo/TF와 추정 결과를 기록할 수 있다.
