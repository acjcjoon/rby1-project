# nvblox 실험: RViz, 스테이션 이동, 지도 저장

카메라를 로봇에서 분리해서 손으로 움직일 때는
[handheld RGB 재구성 절차](handheld_nvblox.md)를 사용한다.
아래 절차는 카메라가 로봇에 장착된 상태의 wheel odometry/로봇 TF를 사용한다.

기존 localization 정책을 그대로 사용한다. 이동 중 wheel odometry,
정지한 스테이션에서 visual correction이며 EKF는 추가하지 않는다.
nvblox는 `odom`에 depth를 누적하고, Nav2 local(`odom`)과 global(`vslam_map`)
costmap에 같은 거리 지도를 제공한다. 자율주행은 전진·회전만 허용한다.

## 환경과 설치

- 호스트: ROS 2 Humble, `/home/nvidia/rby1_ros2_ws`.
- 컨테이너: `rby1-isaac-vslam`, Isaac ROS 3.2.
- nvblox 소스: 컨테이너 `/workspaces/isaac_ros-dev/src/isaac_ros_nvblox`, `release-3.2`.
- 확인한 소스 commit: `7908a183acf84f4f1ab3fda7b6d6caf3eefc1f78`.
- 같은 소스의 호스트 경로: `/mnt/ssd/workspaces/isaac_ros-dev/src/isaac_ros_nvblox`.
- 호스트에서 `nvblox_msgs`, `nvblox_nav2`, `nvblox_rviz_plugin`을 소스 빌드한다.
- 컨테이너에서는 nvblox GPU 노드와 `realsense_splitter`를 빌드한다.
- RViz는 호스트에서 실행한다. 기존 컨테이너에 X11 mount를 추가할 필요가 없다.

현재 호스트의 시스템 librealsense는 emitter metadata를 전달하지 않았다.
`--nvblox`는 이미 존재하는 RSUSB 빌드
`~/librealsense-rsusb-2.58.4/build-rsusb/Release/librealsense2.so.2.58.4`를
카메라 프로세스에만 `LD_PRELOAD`로 적용한다. 시스템 라이브러리·펌웨어는 변경하지 않는다.
경로가 다르면 `--rsusb-library /절대경로/librealsense2.so` 또는
`RBY1_REALSENSE_LIBRARY`로 지정한다. 경로가 없으면 명시적으로 실행을 중단한다.

설치 시 승인한 패키지는 nvblox 3.2.5의 `nvblox-ros`, `nvblox-nav2`,
`nvblox-rviz-plugin`, `nvblox-msgs`, `nvblox-ros-common`, `nvblox-ros-python-utils`와
`realsense2-camera-msgs` 4.58.4뿐이다. 기존 패키지 upgrade/remove와 호스트 apt
설치는 수행하지 않는다. 컨테이너를 삭제·재생성하면 이 apt 설치는 다시 필요하다.
`scripts/install_nvblox_container.sh`는 기본적으로 모의 실행만 하며,
실제 sudo 설치는 사용자의 별도 허가 후에만 실행한다.

코드 변경 후 재빌드:

```bash
# 호스트
cd /home/nvidia/rby1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select rby1_vslam

# 컨테이너의 Python 통합 패키지
docker exec -it rby1-isaac-vslam bash -lc '
  source /opt/ros/humble/setup.bash
  cd /workspaces/isaac_ros-dev
  colcon build --symlink-install --base-paths src/rby1_vslam --packages-select rby1_vslam
'
```

## 실행 순서

아래 터미널마다 호스트 공통 환경을 먼저 불러온다.

```bash
cd /home/nvidia/rby1_ros2_ws/src/rby1-project
source /opt/ros/humble/setup.bash
source /home/nvidia/rby1_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export RBY1_ROS_DOMAIN_ID=10
```

```bash
# 1. 기존 베이스 driver/control + 스테이션 UI
bash rby1_vslam/run_mobile_base.sh start_camera:=false \
  max_linear_speed:=0.1 max_angular_speed:=0.2
```

```bash
# 2. 호스트 카메라: IR/depth 90 Hz, firmware emitter 교대
RBY1_D435_SERIAL=244222075961 bash rby1_vslam/run_d435i_host.sh 0 --nvblox
```

```bash
# 3. 컨테이너: splitter + cuVSLAM + 기존 localization
bash rby1_vslam/run_isaac_vslam_docker.sh 0 \
  use_realsense_splitter:=true camera_fps:=90 start_rviz:=false
```

```bash
# 4. 컨테이너: nvblox. 이 프로세스만 다시 실행해도 localization은 유지된다.
bash rby1_vslam/run_nvblox_docker.sh
```

```bash
# 5. 호스트: global/local nvblox costmap + 전진 전용 RPP
# gate는 정지 상태에서 시작하며 실제 Goto는 기존 UI에서 수행한다.
bash rby1_vslam/run_navigation.sh use_nvblox:=true use_scan:=false \
  start_localization_tf:=false enable:=false
```

```bash
# 6. 호스트 RViz
bash rby1_vslam/run_nvblox_rviz.sh
```

Humble RViz에 GLSL150 자원이 없으면 실행 스크립트가 nvblox의 공식 material
fallback을 자동 선택한다. `rviz/glsl150/box.geom`을 찾지 못해 첫 mesh 수신 시
종료되는 문제를 피하며 mesh·ESDF·costmap 표시는 유지된다. 선택적인 voxel layer의
상자 표시는 점으로 바뀐다. 별도 설치나 빌드는 필요하지 않다.
동일 synthetic mesh/ESDF/costmap으로 기존 실행은 약 2.9초에 종료되고,
수정 후에는 16초간 mesh 157회 발행 중 정상 표시·종료를 확인했다.
`Stereo is NOT SUPPORTED`는 RViz 입체 디스플레이 관련 정보이며 이 오류의 원인이 아니다.

기존 VO-only 카메라/Isaac 프로세스와 중복 실행하지 않는다. 60 Hz로 비교할 때는
터미널 2에 `--camera-fps 60`, 터미널 3에 `camera_fps:=60`을 사용한다.
목표 출력은 90 Hz일 때 IR stereo 45 Hz + depth 45 Hz,
60 Hz일 때 각각 30 Hz다. nvblox ESDF는 10 Hz로 별도로 갱신한다.

아래 두 측정은 DDS 공유 메모리 설정 수정 전의 기준값이다.
2026-10-26 정지 상태 실측에서는 90 Hz 원본 metadata가 약 89.85 Hz,
splitter depth가 42.37 Hz, 좌우 IR이 43.24/44.11 Hz였다. 최대 timestamp 간격은
약 66.8 ms였으며 검증된 프레임의 잘못된 emitter 분류는 없었다.
손실 없는 45 Hz는 아직 확인되지 않았다. 60 Hz 비교에서는 depth 27.34 Hz,
IR 28.85/29.05 Hz로 더 안정적이지 않아 90 Hz를 기본으로 유지한다.
실제 cuVSLAM/nvblox/RViz 동시 실행 중의 지연·드롭은 터미널 통계로 다시 확인한다.

정지한 실제 카메라로 cuVSLAM과 소스 빌드 nvblox를 동시에 구동한 25초 측정에서는
VO odometry 41.06 Hz(유효 pose 999개), cuVSLAM 상태 998/998 성공,
depth 적분 약 38~40 Hz, ESDF 9.23 Hz를 확인했다. 기존 40 ms VO jitter 기준의
프레임 간격 경고는 남아 있으며 설정을 완화하지 않았다. 이 시험은 별도 ROS 도메인의
임시 고정 카메라 TF로 데이터 흐름을 검증한 것이고 실제 이동 정확도 검증은 아니다.

이후 실제 D435i + splitter + median + cuVSLAM + nvblox를 동일 조건으로 비교해,
Fast DDS의 기본 512 KiB 공유 메모리 구간을 32 MiB로 확대한 효과를 확인했다.
각 약 22초의 정상 동작 구간에서 VO는 40.63 → 44.93 Hz,
최대 획득 timestamp 간격은 66.77 → 22.26 ms로 개선됐다.
수정 후 VO 1,006개와 연속 간격 1,005개를 관측했고 40 ms 초과 간격은 없었다.
nvblox 내부 depth 처리율도 약 45 Hz, ESDF 약 9.3 Hz였다.
`image_jitter_threshold_ms: 40.0`은 변경하지 않았다.
수정한 RViz를 실제 카메라·VO·nvblox와 함께 실행한 후속 시험에서도 VO 44.94 Hz,
최대 간격 22.26 ms, 측정한 1,008개 연속 간격에서 40 ms 초과 0회였고
RViz의 geometry shader 예외가 재발하지 않았다.

`config/fastdds_camera.xml`을 카메라 `--nvblox`, Isaac, nvblox 실행 스크립트가
`FASTRTPS_DEFAULT_PROFILES_FILE`로 적용한다. 호스트 카메라는 호스트 환경의 기존 값을,
Docker 실행 스크립트는 컨테이너 환경의 기존 값을 존중한다. 호스트의 파일 경로는
컨테이너에 자동 전달하지 않는다. UDP 통신은 유지하며 시스템 네트워크 설정,
QoS, emitter 주기, localization 정책은 변경하지 않는다.
설정 반영에는 카메라·Isaac·nvblox 프로세스를 다시 시작해야 한다.
Isaac 재시작 후에는 기존 절차대로 스테이션 보정을 다시 수행한다.
이 비교 역시 별도 ROS 도메인의 고정 카메라 TF 시험이며 이동 정확도 검증은 아니다.

## 실험 1: 수동 이동하면서 지도 생성 확인

1. UI에서 기존 순서대로 전원·서보·제어·stream을 준비한다.
2. 베이스와 head를 멈추고 `Stop + correct at this station`으로 초기 보정을 완료한다.
3. RViz의 `TSDF Mesh`, `ESDF Distance Slice`, `Local Costmap`, `Global Costmap`을 확인한다.
4. head가 진행 방향을 보도록 두고 저속 수동 전진·회전으로 출발점과 이동 통로를 관측한다.
5. 벽·상자·의자 다리가 costmap에 반영되는지, 바닥이 장애물로 남지 않는지 확인한다.
6. 베이스 정지 상태에서 head를 움직여도 구조물이 따라 움직이거나 이중으로 생기지 않는지 확인한다.

RViz 기본 fixed frame은 `odom`이다. global costmap은 TF를 통해 같은 화면에 보인다.
`Images`의 depth/OFF IR 표시와 depth cloud는 필요할 때 활성화한다.
높이 범위 표시는 `odom` 기준 0.08~1.8 m이며, 실제 바닥·낮은 장애물·로봇 자세를
확인한 후 `config/nvblox.yaml`에서 조정한다. 이 구간보다 낮거나 높은 물체는
해당 costmap 장애물에 포함되지 않는다.

```bash
ros2 topic hz /rby1/camera/infra1/image_rect_raw
ros2 topic hz /rby1/nvblox/depth
ros2 topic hz /nvblox_node/static_map_slice
ros2 topic hz /nvblox_node/back_projected_depth/d435_depth_optical_frame
ros2 run tf2_ros tf2_echo odom d435_depth_optical_frame
ros2 topic echo /rby1/vslam/navigation_status --once
```

큰 Image의 `ros2 topic hz`는 계측 자체의 부하 영향을 받는다. 원본 timestamp 간격,
nvblox 터미널의 integration rate/delay/queue-drop 통계를 함께 확인한다.
splitter 출력이 없으면 emitter metadata와 카메라 로그를 먼저 확인한다.
카메라 입력이 정상이어도 integrated cloud가 없으면 촬영 시각의 TF 또는 depth 적분을 확인한다.

현재 설정은 `static_tsdf`다. 물체를 치웠다면 원래 위치를 다시 관측해서 지워지는지
확인한다. 미관측 영역을 시간만으로 free로 바꾸지 않는다. 로봇으로부터 반경 12 m
밖의 voxel은 제거하므로 저장물도 현재 유지 중인 영역만 포함한다.
`static_mapper.do_depth_preprocessing=false`는 유지한다. 현재 3.2의 depth-only
경로에서 이 옵션을 켜면 빈 mask assertion으로 첫 적분 시 종료되는 것을 재현했다.

## 실험 2: UI의 스테이션 이동과 장애물 회피

1. 먼저 수동으로 두 스테이션 및 우회할 통로를 관측한다. unknown은 통과하지 않는다.
2. 각 스테이션에서 정지·보정 후 `Save current localization`으로 위치를 저장한다.
3. 출발점으로 돌아와 정지·보정한 뒤, 넓은 통로의 중간에 정적인 장애물을 놓는다.
4. RViz에서 장애물 표시와 주변 inflation이 생긴 것을 확인한다.
5. UI에서 목적 스테이션을 선택해 Goto한다. `Global Plan`이 우회하고 베이스가
   방향을 맞춘 후 전진하는지 확인한다. 도착 후 기존 스테이션 보정을 수행한다.
6. 우회 통로가 없는 경우 정지/경로 실패가 정상이다. unknown 허용이나 costmap
   삭제로 통과시키지 않는다. 현재 BT는 1 Hz로 재계획하며 자동 후진 recovery는 없다.

현재 global costmap은 20×20 m rolling window다. 창 밖 목표나 관측되지 않은
우회 경로로는 이동할 수 없다. 좁은 공간에서는 사각형 베이스의 회전 범위도 확보해야 한다.
전진 전용 제약은 Nav2 자율주행에 적용하며 기존 수동 UI의 조작 방식은 변경하지 않는다.

gate는 depth, ESDF slice, 실제 적분 후 발행되는 cloud의 신선도를 모두 확인한다.
카메라 단절·적분 정지·시간 역행·역방향 명령 시 정지하고 기존 goal을 취소한다.
원인을 해결한 후 UI에서 다시 Goto해야 한다. 처음 출발점의 footprint나 회전 영역이
unknown이면 controller가 멈출 수 있으므로 사전 관측이 필요하다.

## 실험 3: 지도 저장·확인

서로 다른 저장물을 구분한다.

| 저장물 | 용도 | 좌표계/재사용 |
|---|---|---|
| cuVSLAM `.mdb` 폴더 | 다음 실행의 visual localization | 기존 map_tool 절차 |
| nvblox `.nvblx` + `.json` | 현재 static voxel 지도 저장·동일 세션 복원 | 현재 wheel `odom` 기준 |
| `.ply` | 3D 구조 확인·외부 뷰어 | navigation 지도 자체가 아님 |
| waypoint YAML | 스테이션 위치 | `vslam_map` 기준, 기존 UI |

UI에서 주행 취소 후 `cancel_pending=false`, `active_goal_count=0`을 확인한다.
`Stop + correct at this station`의 보정을 마친 뒤 베이스와 head를 계속 정지시킨다.
저장 이름은 매 실험마다 바꾼다. 기존 파일은 덮어쓰지 않는다.

```bash
docker exec -it -e ROS_DOMAIN_ID=10 rby1-isaac-vslam bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
mkdir -p /workspaces/isaac_ros-dev/maps/nvblox_trial_01

ros2 run rby1_vslam map_tool save \
  --path /workspaces/isaac_ros-dev/maps/nvblox_trial_01/visual
ros2 run rby1_vslam nvblox_map_tool save \
  --path /workspaces/isaac_ros-dev/maps/nvblox_trial_01/scene.nvblx
ros2 run rby1_vslam nvblox_map_tool mesh \
  --path /workspaces/isaac_ros-dev/maps/nvblox_trial_01/scene.ply
```

서비스 성공 응답과 실제 비어 있지 않은 파일을 도구가 확인한다. `.nvblx.json`에는
localization 세션, 좌표계, voxel 크기, mapping type을 남긴다. waypoint는 기존
`rby1_vslam/data/rby1_vslam_waypoints.yaml` 또는 UI에 지정한 별도 파일을 보관한다.
컨테이너 `maps`는 호스트 `/mnt/ssd/workspaces/isaac_ros-dev/maps`에 남는다.

복원 확인은 **베이스 driver와 localization을 유지한 같은 세션에서** 수행한다.
주행을 정지한 상태에서 터미널 4의 nvblox만 종료·재실행한 후:

```bash
ros2 run rby1_vslam nvblox_map_tool load \
  --path /workspaces/isaac_ros-dev/maps/nvblox_trial_01/scene.nvblx
```

RViz에서 geometry가 돌아오는지 확인한다. 카메라 적분은 재개되므로 저장 이후
환경 변화는 새 관측으로 반영된다. wheel odom 또는 localization을 재시작한 후에는
기존 `.nvblx`를 같은 위치라고 가정하지 않는다. 도구가 다른 세션의 load를 거부한다.
visual map을 재localize하더라도 예전 `odom` voxel 전체가 자동 정합되는 것은 아니다.
이번 구성은 영속 전역 occupancy 지도와 loop closure 재통합까지 구현하지 않는다.

## 설정 위치

- `config/nvblox.yaml`: voxel, depth 적분/ESDF 주기, 장애물 높이, 지도 유지 반경.
- `config/d435i_host_nvblox.yaml`: 호스트 emitter 교대와 sensor QoS.
- `config/navigation_nvblox.yaml`: 전진 전용 RPP, 속도 제한, global/local nvblox layer.
- `config/nvblox.rviz`: 디버깅 화면.
- `use_nvblox:=false`: 기존 Nav2 구성으로 실행. 기존 localization 설정은 공통이다.

## 검증 범위

2026-10-26에 다음을 확인했다. 아래 합성 입력 시험은 실제 로봇 도메인과 분리했다.

- 호스트 nvblox 메시지/costmap/RViz 플러그인과 컨테이너 GPU 노드/splitter 소스 빌드 성공.
- 정지 상태 D435i의 emitter metadata와 OFF IR / ON depth 분리 확인.
- 실제 카메라 → splitter → median/cuVSLAM과 nvblox 동시 실행, 유효 VO와 ESDF 동시 출력 확인.
- 소스 빌드 GPU 노드에 합성 depth를 넣어 장애물 224셀, 자유 공간 1,066셀의 ESDF 생성 확인.
- 실제 nvblox save/load/PLY 서비스 및 비어 있지 않은 저장 파일 확인.
- Nav2 7개 lifecycle node active, global/local nvblox plugin 및 전진 전용 RPP 로드 확인.
- 가상 벽을 넣은 ComputePathToPose가 우회 경로 생성. FollowPath/실제 이동은 실행하지 않음.
- 호스트 RViz에서 준비한 설정과 표시 플러그인 정상 실행.
- 카메라 CLI, 지도 저장 도구, costmap 설정/정지 감시 회귀 테스트 48개 통과.

실제 모바일 베이스의 이동·장애물 회피·스테이션 도착 오차는 위 실험 절차로 확인할 대상이다.
