# RBY1 Jetson VSLAM

`main`에서 시작한 `VSLAM` 브랜치에 `feat/VSLAM`의 `rby1_vslam`만 가져왔다.
SSD가 장착된 Jetson에서 Isaac ROS를 직접 실행한다. 영상·IMU·위치는 같은 Humble
ROS 그래프에서 전달된다. 다른 패키지와 저장소 루트 런처는 수정하지 않는다.

```text
RBY1 U-PC: Jetson AGX Orin 64GB / AVerMedia D315
설치 대상: JetPack 6.1 또는 6.2 / Ubuntu 22.04 / ROS 2 Humble
  D435i IR1 + IR2 (+ IMU) → Isaac ROS 3.2 cuVSLAM
                                  ├→ tracking_monitor → tracking_status
                                  └→ pose_adapter → base SLAM pose
                                                       ↓
  robot driver → wheel odom/TF → localization_tf → vslam_map → odom
                                                       ↓
  Nav2 → nav2_gate → /rby1/cmd_raw → 기존 rby1_control → /rby1/cmd_vel
  SSD: Isaac workspace, Docker data, cuVSLAM 지도, rosbag, waypoint
```

기존 프로젝트의 Humble에 맞춰 **Isaac ROS release-3.2**를 선택했다.
사용 대상은 RBY1 내장 U-PC다. [Rainbow Robotics 하드웨어 문서](https://rainbowrobotics.github.io/rby1-dev/hardware/electrical_part.html#u-pc-and-r-pc)는
Jetson AGX Orin 64GB와 AVerMedia D315 캐리어보드를 명시한다.
현재 설치된 JetPack 버전은 로봇 모델명만으로 확정할 수 없어 설치 전에 확인한다.
이 Isaac 조합의 지원 대상은
[NVIDIA 지원 환경](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html)에
따른 Jetson Orin이다. Xavier나 JetPack 5에는 이 조합을 그대로 설치하지 않는다.
설치는 [Jetson SSD 가이드](docs/jetson_physical.md)를 따른다.

## 실행

Isaac ROS 3.2 환경에서 driver/description/control 패키지까지 빌드했다면:

```bash
source /opt/ros/humble/setup.bash
source "$ISAAC_ROS_WS/install/setup.bash"
export ROS_DOMAIN_ID=0
ros2 launch rby1_vslam physical.launch.py tracking_mode:=0
```

`bash rby1_vslam/run_vslam.sh tracking_mode:=0`도 같은 launch를 실행한다.
`RBY1_WORKSPACE_SETUP`으로 다른 workspace의 `install/setup.bash`를 지정할 수 있다.
기존 로봇 노드를 Jetson 호스트에서 계속 사용하면 컨테이너에서 다음과 같이 실행한다.

```bash
ros2 launch rby1_vslam physical.launch.py tracking_mode:=0 \
  start_driver:=false start_description:=false start_control:=false
```

호스트와 컨테이너 모두 Humble, 같은 `ROS_DOMAIN_ID`와 호환 DDS 설정을 사용하고
컨테이너는 host network로 실행한다. 호스트의 wheel odom, joint states, 로봇 TF,
control 서비스가 컨테이너에서도 보이는지 먼저 확인한다.
기존 카메라가 같은 토픽을 발행하면 `start_camera:=false`, mount TF가 이미 있으면
`publish_mount_tf:=false`도 지정한다. GUI 없는 환경에서는 `start_ui:=false`를 사용한다.
전원·서보·control manager·stream은 UI에서 켠다. Nav2 gate는 시작 시 OFF다.

## 입력 모드

| tracking_mode | D435i 입력 | Isaac ROS 3.2 |
|---|---|---|
| `0` | IR stereo, 기본 640×360 @ 30 Hz | stereo VO/SLAM |
| `1` | IR stereo + united IMU | `enable_imu_fusion=true` |
| `2` | color + aligned depth | 녹화 전용; VSLAM launch에서 거부 |

3.2의 API에 맞춰 4.5용 `tracking_mode`/RGB-D 설정을 제거했다.
launch는 Isaac 3.2.x와 카메라의 `release/4.51.1-isaac` 패키지 버전을 검사한다.
이 RealSense 버전은 IR/depth가 `depth_module.profile`을 공유하므로
`publish_all_sensors:=true`를 쓰면 `infra_profile`과 `depth_profile`을 같게 지정한다.
새 카메라 노드는 `/d435/d435/d435`, 영상 토픽은 기존 `/d435/d435/...`를 유지한다.

## 지도와 Point

지도는 cuVSLAM과 같은 Jetson/컨테이너에서 보이는 SSD 경로를 사용한다.
먼저 UI의 Cancel 또는 `/rby1/vslam/cancel`로 navigation을 멈춘다.

```bash
mkdir -p "$ISAAC_ROS_WS/maps"
ros2 run rby1_vslam map_tool save --path "$ISAAC_ROS_WS/maps/rby1_01"
```

저장 대상은 새 디렉터리 또는 빈 디렉터리여야 한다. 3.2의 완료 응답과 생성된 `.mdb`
데이터베이스를 검사한다. 다음 실행은 같은 `tracking_mode`와 저장 지도로 시작한다.

```bash
ros2 launch rby1_vslam physical.launch.py mode:=localization tracking_mode:=0 \
  map_path:="$ISAAC_ROS_WS/maps/rby1_01"

# 영상이 들어온 뒤 다른 Isaac 환경 터미널에서 실행한다.
ros2 run rby1_vslam map_tool localize --path "$ISAAC_ROS_WS/maps/rby1_01" \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
```

힌트는 `vslam_map`에서의 `d435_link` 위치·자세다. 위 0 값은 지도 생성 시작 자세의
예시다. localization 모드에서는 `localized_in_exist_map=Yes`가 확인돼야 navigation을
활성화할 수 있다. 이전 4.5 지도와의 호환은 검증하지 않았으므로 3.2 지도를 새로 생성한다.

cuVSLAM 특징점 지도, waypoint YAML, Nav2 occupancy map은 별도 파일이다.
waypoint 기본 경로는 `data/rby1_vslam_waypoints.yaml`이며
`waypoints_file:=/SSD/절대경로/points.yaml`로 바꿀 수 있다.
`vslam_map`에 정렬된 occupancy map은 `occupancy_map:=/절대경로/map.yaml`로 지정한다.
기본 `use_scan:=false`에는 장애물 감지·회피가 없다.
LiDAR가 실행 중이면 `use_scan:=true scan_topic:=/scan`을 지정한다.

## 녹화와 재생

```bash
# 카메라/수동 이동만 실행; 두 명령의 번호를 맞춘다.
bash rby1_vslam/run_mobile_base_mode1.sh
bash rby1_vslam/recorder.sh mode1
```

mode0/1/2 녹화를 지원한다. RGB-D mode2는 3.2 VSLAM 입력이 아니다.
IMU 효과 비교에는 mode1 bag을 0과 1로 각각 재생한다.

```bash
ros2 launch rby1_vslam isaac.launch.py tracking_mode:=1 use_sim_time:=true start_rviz:=true
ros2 bag play /path/to/mode1_bag --clock
```

bag에는 camera info와 `/tf_static`이 있어야 한다. 베이스 위치까지 변환하려면
`jetson.launch.py start_camera:=false use_sim_time:=true`를 사용하고 녹화 당시 관절·로봇
TF도 공급한다. 녹화 런처는 robot_state_publisher를 시작하지 않으므로 기존 로봇 TF
발행기가 필요하다. 실물 navigation launch는 wall clock을 사용한다.

## TF와 상태

cuVSLAM은 움직이는 헤드의 `d435_link`를 추적하고, `pose_adapter`가 촬영 시점의
`d435_link → base` TF로 베이스 위치를 계산한다. Isaac TF 출력은 꺼서 카메라 부모
중복을 방지한다. `localization_tf`만 `vslam_map → odom`을 발행하고 driver의
`odom → base_footprint → base` 경로를 사용한다.

```bash
ros2 topic hz /d435/d435/infra1/image_rect_raw
ros2 topic echo /rby1/vslam/tracking_status --once
ros2 topic echo /rby1/vslam/localization_status --once
ros2 topic echo /rby1/vslam/navigation_status --once
ros2 run tf2_ros tf2_echo vslam_map base
```

상태와 camera odometry의 촬영 timestamp·수신 시간을 검사한다.
추적 손실·상태 만료·tracker reset·세션 변경 시 gate가 정지하고 목표를 취소한다.
복구 후에도 명시적으로 다시 켜야 하며 오래된 속도 명령을 내보내지 않는다.
기본 mount 값은 기존 카메라 설정에서 가져왔다. `mount_x/y/z/roll/pitch/yaw`로
실측 장착값을 지정한다. 각도 단위는 radian이다.

## 개발 PC 검증

```bash
PYTHONPATH=rby1_vslam python3 -m unittest discover -s rby1_vslam/test -v
```

ROS 없이 freshness, 정지·재활성화, TF 소유권, camera profile, 지도 경로와 launch 연결을
검증한다. ROS component 로딩, GPU tracking, USB, 실제 navigation은 Jetson에서 확인한다.
