# RBY1 VSLAM 실행 명령

**nvblox 실험:** [설치 상태, RViz 디버깅, 스테이션 회피 주행, 지도 저장 절차](docs/nvblox_experiment.md).
**분리 카메라 RGB 실험:** [VSLAM 카메라 포즈 + 컬러 nvblox mesh](docs/handheld_nvblox.md).
기존 wheel 이동 / 정지 시 visual 보정 정책을 유지하며, `use_nvblox:=true`에서만
nvblox global/local costmap과 전진 전용 RPP를 사용한다.

Jetson 호스트: ROS 2 Humble, `/home/nvidia/rby1_ros2_ws`.
Isaac ROS 3.2 컨테이너: `rby1-isaac-vslam`, `/workspaces/isaac_ros-dev`.
모든 터미널은 `ROS_DOMAIN_ID=10`을 사용한다.

## 빌드

```bash
cd /home/nvidia/rby1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select rby1_omni_controller rby1_vslam
source install/setup.bash

docker exec -it rby1-isaac-vslam bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash
  cd /workspaces/isaac_ros-dev
  colcon build --symlink-install --packages-select rby1_vslam
'
```

Omni 경로 추종 플러그인 `rby1_omni_controller`는 Nav2가 실행되는 호스트에서 빌드한다.
컨테이너는 cuVSLAM과 localization만 실행하므로 이 C++ 플러그인 빌드가 필요 없다.

## 한 번에 실행 (터미널 1개)

아래 터미널 1~4를 순서대로 띄우고, 각 단계의 토픽을 확인한 뒤 다음을 시작한다.
로그는 `[base]` `[cam]` `[vslam]` `[nav]`가 붙어 한 화면에 나오고 `~/rby1_bags/logs/stack_<시각>/`에 저장된다.
`Ctrl-C` 한 번이면 역순(Nav2 → VSLAM → 카메라 → 베이스)으로 정리하며, 컨테이너 안의 cuVSLAM도 함께 끈다.
하나가 종료되면 나머지도 정리된다.

```bash
cd /home/nvidia/rby1_ros2_ws/src/rby1-project
bash rby1_vslam/run_station_stack.sh                      # 90 fps, ~/rby1_bags/exp3_stations.yaml
bash rby1_vslam/run_station_stack.sh --fps 30             # IR 30 fps
bash rby1_vslam/run_station_stack.sh --record exp3_vslam  # bag 기록 포함
bash rby1_vslam/run_station_stack.sh --no-nav             # Nav2 없이 (수동 주행 실험)
bash rby1_vslam/run_station_stack.sh --dry-run            # 실행할 명령만 출력
```

## 실행: 호스트 터미널 4개

각 터미널에서 공통으로 실행한다.

```bash
cd /home/nvidia/rby1_ros2_ws/src/rby1-project
source /opt/ros/humble/setup.bash
source /home/nvidia/rby1_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export RBY1_ROS_DOMAIN_ID=10
```

```bash
# 1. 베이스 driver/control + 스테이션 UI
bash rby1_vslam/run_mobile_base.sh start_camera:=false \
  max_linear_speed:=0.1 max_angular_speed:=0.2
```

```bash
# 2. 호스트 D435i: IR stereo 640×480@90, projector OFF (--camera-fps 30|60 으로 변경 가능)
RBY1_D435_SERIAL=244222075961 bash rby1_vslam/run_d435i_host.sh 0
```

```bash
# 3. Docker: cuVSLAM + VO 베이스 변환 + 스테이션 localization
bash rby1_vslam/run_isaac_vslam_docker.sh 0 start_rviz:=false
```

```bash
# 4. 호스트 Nav2: localization은 터미널 3에서만 실행
bash rby1_vslam/run_navigation.sh \
  start_localization_tf:=false use_scan:=false
```

첫 실행은 `mapping` 모드다. UI에서 전원·서보·제어·stream을 준비하고 수동으로
이동하며 지도를 만든다. 각 스테이션에서 베이스와 헤드를 정지한 뒤
`Stop + correct at this station` → 보정 완료 → `Save current localization`으로
포인트를 저장한다. 아래 명령으로 지도도 저장한 뒤 선택한 포인트로 Goto한다.
이동 중에는 마지막 `vslam_map → odom` 보정값을 고정하고 휠 오도메트리로 추정한다.
도착 후에는 정지와 VO 수렴을 확인해 한 번 보정하며, 작업 후 다음 Goto에서 이동 모드로 전환한다.
보정 조건은 `config/localization.yaml`, 속도·도착 허용 오차는 `config/navigation.yaml`에 있다.
Nav2는 Omni Pursuit 제어기로 출발 시 몸체 방향을 유지하며 전후·좌우로 경로를 따라가고,
목표 위치에서 멈춘 뒤 저장된 yaw를 맞춘다. 몸체 기준 `+X` 전진, `+Y` 좌측, `+yaw` 반시계 방향이다.
현재 드라이버의 odom 좌표계 속도는 `/rby1/vslam/wheel_odom`에서 몸체 좌표계로 변환해 Nav2에 제공한다.
몸체 좌표계 twist를 발행하는 다른 드라이버·시뮬레이터는 터미널 4에 `wheel_twist_frame:=body`를 추가한다.

Waypoint는 `data/rby1_vslam_waypoints.yaml`에 저장한다. 다른 파일은 터미널 1에
`waypoints_file:=/절대경로/stations.yaml`을 추가한다. 재실행 시 같은 지도 좌표계를 사용한다.
`use_scan:=false`에서는 장애물 감지·회피가 없다. LiDAR를 연결한 경우 터미널 4에
`use_scan:=true scan_topic:=/scan`을 사용한다. nvblox는 위 별도 실험 절차로 실행한다.

## 상태 확인·정지·보정

```bash
ros2 topic echo /rby1/vslam/localization_status --once
ros2 topic echo /rby1/vslam/navigation_status --once
ros2 topic echo /rby1/vslam/pose_adapter_status --once  # Base VO 변환·TF 실패 원인
ros2 topic echo /rby1/vslam/camera_odometry --once  # 카메라 VO XYZ/자세
ros2 topic echo /rby1/vslam/localization_odom --once  # 베이스 추정 위치
ros2 run tf2_ros tf2_echo vslam_map base

# 주행 취소 후 status의 cancel_pending=false, active_goal_count=0 확인
ros2 service call /rby1/vslam/cancel std_srvs/srv/Trigger '{}'
# 정지한 스테이션에서 보정 요청; station_converged=true 확인
ros2 service call /rby1/vslam/set_station_mode std_srvs/srv/SetBool '{data: true}'
# 이동 모드: 보정값 고정 (실제 Goto와 gate 활성화는 UI에서)
ros2 service call /rby1/vslam/set_station_mode std_srvs/srv/SetBool '{data: false}'
```

`localization not initialized`이면 정지 후 UI의 `Stop + correct at this station`을 누른다.
`cancel_pending=true`가 계속되면 Nav2 터미널에서 시작 실패를 확인한다.
`OmniPursuitController ... does not exist`는 새 플러그인 경로가 실행 환경에 없는 경우다.
호스트 빌드 후 해당 Nav2를 종료하고 위 `run_navigation.sh`로 다시 실행한다.
이 스크립트는 매번 호스트 workspace 환경을 다시 불러온다.

## 지도 저장·다음 실행에서 사용

주행을 취소하고 정지한 뒤 저장한다. 저장 경로는 컨테이너에서 보이는 새 폴더다.

```bash
docker exec -it -e ROS_DOMAIN_ID=10 rby1-isaac-vslam bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash
  mkdir -p /workspaces/isaac_ros-dev/maps
  ros2 run rby1_vslam map_tool save --path /workspaces/isaac_ros-dev/maps/stations
'

# 다음 실행의 터미널 3 (기존 VSLAM 종료 후)
bash rby1_vslam/run_isaac_vslam_docker.sh 0 mode:=localization \
  map_path:=/workspaces/isaac_ros-dev/maps/stations
```

영상이 들어오면 별도 터미널에서 저장 지도 위치 인식을 요청한다.
아래 pose hint는 예시이므로 `vslam_map` 기준 현재 `d435_link`의 XYZ(m)와 yaw(rad)로 바꾼다.

```bash
docker exec -it -e ROS_DOMAIN_ID=10 rby1-isaac-vslam bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash
  ros2 run rby1_vslam map_tool localize --path /workspaces/isaac_ros-dev/maps/stations \
    --x 0.0 --y 0.0 --z 1.5 --yaw 0.0
'
```

위치 인식 완료 후 UI에서 스테이션 보정을 요청하고, 보정이 끝나면 저장된 포인트로 Goto한다.
VO가 재시작되어 좌표계가 바뀐 경우 정지·스테이션 모드에서 아래 명령으로 새 VO를
현재 휠 추정 좌표에 연결한다. 이 작업은 재시작 동안 누적된 휠 오차를 복구하지 않는다.

```bash
ros2 service call /rby1/vslam/reset_visual_reference std_srvs/srv/Trigger '{}'
```

## 영상 확인·녹화

```bash
ros2 run rqt_image_view rqt_image_view /d435/d435/infra1/image_rect_raw
ros2 topic hz /d435/d435/infra1/image_rect_raw
RBY1_RECORD_VSLAM=true bash rby1_vslam/recorder.sh mode0
```
