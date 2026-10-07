# VSLAM mapping 왕복·정지 jitter 기록

로봇을 처음과 끝에 완전히 정지시키고, mapping 경로를 돈 뒤 같은 자리로 돌아오는
실험을 위한 수동 구간 표식 수집이다. 수집기는 로봇 명령, enable, cancel, map save를
호출하지 않는다.

## 기록 토픽

기본 기록은 원본 IR 영상을 제외한 저부하 프로필이다.

| 구분 | 토픽 | 확인 항목 |
|---|---|---|
| 카메라 입력 | `/d435/d435/infra{1,2}/camera_info` | stereo frame stamp/rate/gap |
| IMU | `/d435/d435/imu` | 정지 중 gyro와 acceleration 진동 |
| cuVSLAM 원본 | `/rby1/vslam/camera_odometry`, `/rby1/vslam/camera_slam_odometry` | camera tracking/map pose |
| base 변환 결과 | `/rby1/vslam/odom`, `/rby1/vslam/slam_odom` | 실제 navigation에 쓰는 base pose |
| 비교 기준 | `/rby1/odom`, `/tf`, `/tf_static` | wheel odometry와 map→odom 보정 |
| 상태·원인 | `/visual_slam/status`(LAB), `/rby1/vslam/bridge_status`, `/rby1/vslam/timing`, `/diagnostics`, `/rosout` | tracking loss, queue/drop, 처리 지연 |
| 실제 명령 | `/rby1/vslam/nav2_cmd_vel`, `/rby1/cmd_raw`, `/rby1/cmd_vel` | 정지 판정과 의도치 않은 명령 |

원본 좌우 IR 영상은 기록량과 DDS/디스크 부하가 VSLAM 자체에 jitter를 만들 수 있어
기본값에서 제외한다. 영상 자체가 의심되는 짧은 별도 시험에만 `--full-bag`을 쓴다.

## 실행

먼저 VSLAM과 로봇 stack을 실행하고 `bridge_status`의 `connected=true`,
`tracking_ok=true`를 확인한다. 그 다음 Jetson의 별도 터미널에서 실행한다.

```bash
cd "$HOME/rby1_ros2_ws/src/rby1-project"
export ROS_DOMAIN_ID=10
bash rby1_vslam/scripts/capture_vslam_mapping_debug.sh \
  --run-id mapping_loop_01
```

스크립트는 다음 순서로 Enter 입력을 기다린다.

1. `initial_stationary`: 로봇과 머리를 최소 10초 완전히 정지한다.
2. Enter를 누른 직후 천천히 mapping을 시작한다.
3. 시작 위치와 시작 방향으로 돌아와 완전히 멈춘 뒤 Enter를 누른다.
4. `returned_stationary`: 다시 최소 10초 정지한 뒤 마지막 Enter로 종료한다.

시작/복귀 때 카메라가 달린 머리 자세도 동일하게 유지한다. 바닥에 base 위치와 방향을
표시하면 수동 복귀 오차와 VSLAM loop-closure 오차를 덜 섞게 된다. 마지막 Enter 전에는
VSLAM/카메라 stack을 먼저 끄지 않는다. 중간에 중단해야 하면 Ctrl+C를 누르면 확보된
구간까지만 안전하게 마무리한다.

LAB의 TCP·GPU·cuVSLAM 처리 지연까지 함께 보려면 같은 trial 동안 LAB 컨테이너에서도
기존 수집기를 실행한다.

```bash
export ROS_DOMAIN_ID=85
bash scripts/capture_vslam_lab.sh --run-id mapping_loop_01
```

## 결과

기본 결과 폴더는
`~/rby1_trials/mapping_loop_01_upc_<시간>_<고유값>/`이다.

| 파일 | 내용 |
|---|---|
| `markers.jsonl` | initial/mapping/returned/end의 local monotonic 시각 |
| `events.jsonl` | pose, velocity, IMU, 상태, 로그의 작은 수치 메타데이터 |
| `bag/` | 같은 저부하 토픽의 원본 ROS 2 bag |
| `timing_report.txt` | 구간별 p95/max 정지 jitter, 시작↔복귀 중심 오차, tracking/drop 진단 |
| `motion_pose_samples.csv` | pose source별 x/y/yaw/속도와 phase |
| `motion_imu_samples.csv` | gyro/acceleration과 phase |
| `plots/mapping_pose_trajectory.png` | 초기 중심을 원점으로 한 tracking/SLAM/wheel 궤적 |
| `plots/stationary_pose_imu_jitter.png` | 초기·복귀 정지 구간 pose/IMU jitter 시계열 |

`matplotlib`이 있으면 PNG는 종료 시 자동 생성된다. 없으면 수집에는 영향이 없고 나중에
다음 명령으로 생성한다.

```bash
python3 rby1_vslam/scripts/plot_vslam_timing.py \
  --report /path/to/timing_report.json \
  --output-dir /path/to/plots
```

`return error`는 “VSLAM 오차”만을 의미하지 않는다. 수동 복귀 위치·방향 오차도 포함한다.
`/rby1/vslam/slam_odom`과 `/rby1/vslam/odom`의 복귀 오차를 `/rby1/odom`과 비교한다.
wheel도 같은 방향으로 어긋나면 실제 복귀 오차 가능성이 크고, wheel은 거의 0인데
SLAM pose만 어긋나면 loop closure/추정/TF를 우선 확인한다. 정지 구간에서 wheel 속도와
명령이 0인데 SLAM pose p95가 1 cm 또는 1 deg를 넘으면 해당 시점의
`/visual_slam/status`, `bridge_status`, `/tf`, `/rosout`을 함께 본다.
