# RBY1 VSLAM 4조건 재생 실험 가이드

## 1. 실험 목적과 조건

실제 로봇에서는 센서 입력을 한 번만 녹화한다.

1. 처음 10초 완전 정지
2. 제자리에서 같은 방향으로 두 바퀴 회전
3. 완전히 멈춘 뒤 마지막 10초 정지

그 뒤 동일한 source bag을 네 번 재생한다. Nav2는 사용하지 않으며, 조건마다 LAB
cuVSLAM을 빈 map 상태로 다시 시작한다.

| 번호 | condition | 두 바퀴 회전까지 Stereo | 마지막 정지 10초 Stereo | IMU | LAB `enable_imu` |
|---:|---|---|---|---|---|
| 1 | `baseline` | 녹화 영상 | 녹화 영상 | 녹화 IMU | `true` |
| 2 | `no_imu` | 녹화 영상 | 녹화 영상 | 사용 안 함 | `false` |
| 3 | `fixed_stereo` | 녹화 영상 | 정지 직후 pair 고정 | 녹화 IMU | `true` |
| 4 | `fixed_stereo_no_imu` | 녹화 영상 | 정지 직후 pair 고정 | 사용 안 함 | `false` |

3·4번도 회전 중에는 원본 stereo sequence를 그대로 사용한다. 두 바퀴 회전이 끝난 뒤
source bag의 `returned_stationary` 마커가 들어오면, relay가 그 다음 정상적인 좌·우
stereo pair와 CameraInfo를 잡아 마지막 구간 동안 반복 발행한다. 좌우에 같은 이미지를
복사하는 방식이 아니다.

이 구성으로 다음을 직접 비교할 수 있다.

- 1 ↔ 2: 전체 운동 구간에서 IMU 사용 효과
- 1 ↔ 3: IMU를 켠 상태에서 마지막 정지 영상의 변화가 만드는 jitter
- 2 ↔ 4: IMU를 끈 상태에서 마지막 정지 영상의 변화가 만드는 jitter
- 3 ↔ 4: 영상이 완전히 고정된 상태에서 IMU가 만드는 pose 변화

---

## 2. 최초 한 번: 코드 동기화와 빌드

UPC와 LAB에 같은 버전의 프로젝트가 있어야 한다. 새 relay를 실제 실행하는 UPC에서:

```bash
cd "$HOME/rby1_ros2_ws"
colcon build --packages-select rby1_vslam --symlink-install
source install/setup.bash

ros2 pkg executables rby1_vslam | grep sensor_replay_relay
```

마지막 명령에서 `sensor_replay_relay`가 보여야 한다.

---

## 3. 실제 센서 source bag 한 번 녹화

### 준비 상태

- UPC `ROS_DOMAIN_ID=10`
- D435i IR 좌/우 영상, CameraInfo, IMU 정상 발행
- mobile base 수동 조작 가능
- Nav2 중지
- 카메라가 달린 머리와 상체 자세 고정
- 문을 닫고 사람이 카메라 앞을 지나가지 않도록 환경 유지
- 저장 공간 수 GB 확보

UPC에서 평소 D435 노드와 수동 base 조작기만 실행한다. cuVSLAM은 source 입력을
녹화하는 데 필수는 아니다. 별도 터미널에서:

```bash
cd "$HOME/rby1_ros2_ws/src/rby1-project"

bash rby1_vslam/scripts/capture_vslam_replay_input.sh \
  --run-id spin2_source \
  --domain 10
```

화면 안내에 따라 아래 순서를 지킨다.

1. 로봇과 카메라를 완전히 정지시키고 Enter
2. 초기 정지 10초가 자동으로 끝날 때까지 대기
3. 같은 방향으로 제자리 두 바퀴 회전
4. 두 바퀴 후 완전히 멈추고 Enter
5. 마지막 정지 10초가 자동으로 끝날 때까지 아무것도 움직이지 않기

특히 4번의 Enter가 `returned_stationary` 마커다. 3·4번 조건은 이 마커 직후의 실제
stereo pair를 고정하므로, 회전 중이거나 아직 흔들리는 상태에서 Enter를 누르면 안 된다.

### source bag 확인

```bash
SOURCE_DIR="$(ls -td "$HOME"/rby1_vslam_replay_inputs/spin2_source_sensor_* | head -n 1)"
SOURCE_BAG="$SOURCE_DIR/bag"

echo "$SOURCE_BAG"
ros2 bag info "$SOURCE_BAG"
sed -n '1,20p' "$SOURCE_DIR/markers.jsonl"
```

`ros2 bag info`에 다음 토픽이 모두 있어야 한다.

```text
/d435/d435/infra1/image_rect_raw
/d435/d435/infra2/image_rect_raw
/d435/d435/infra1/camera_info
/d435/d435/infra2/camera_info
/d435/d435/imu
/tf_static
/rby1/vslam/replay_marker
```

`markers.jsonl`에는 아래 네 phase가 순서대로 있어야 한다.

```text
initial_stationary
rotation_start
returned_stationary
capture_end
```

원본 녹화가 끝나면 live D435와 기존 UPC VSLAM/bridge process를 모두 종료한다.

---

## 4. 조건 하나를 실행하는 공통 순서

두 PC를 사용하므로 안전하게 하나의 shell 명령으로 합칠 수는 없다. 대신 아래 네
터미널 블록을 그대로 복사해 실행한다.

- LAB-A: 새 cuVSLAM
- UPC-A: sensor relay, bridge, rosbag playback
- UPC-B: UPC recorder
- LAB-B: LAB recorder

### 4-1. LAB-A — cuVSLAM 새로 시작

1번 `baseline` 또는 3번 `fixed_stereo`:

```bash
cd /workspaces/isaac_ros-dev/src/rby1-project

bash rby1_vslam/scripts/run_vslam_lab.sh \
  --mode mapping \
  --enable-imu true \
  --no-rviz
```

2번 `no_imu` 또는 4번 `fixed_stereo_no_imu`:

```bash
cd /workspaces/isaac_ros-dev/src/rby1-project

bash rby1_vslam/scripts/run_vslam_lab.sh \
  --mode mapping \
  --enable-imu false \
  --no-rviz
```

LAB 출력에서 아래 값을 확인한다.

```text
ROS_DOMAIN_ID=85
mode=mapping
IMU=true 또는 IMU=false
```

### 4-2. UPC-A — replay process 준비

`COND`만 이번 실험 조건으로 바꾼다.

```bash
cd "$HOME/rby1_ros2_ws/src/rby1-project"

SOURCE_DIR="$(ls -td "$HOME"/rby1_vslam_replay_inputs/spin2_source_sensor_* | head -n 1)"
SOURCE_BAG="$SOURCE_DIR/bag"
COND=baseline
LAB_IP=192.168.0.27    # 실제 LAB IP로 변경

bash rby1_vslam/scripts/run_vslam_replay_condition.sh \
  --bag "$SOURCE_BAG" \
  --condition "$COND" \
  --lab-host "$LAB_IP" \
  --domain 10
```

다음 문구에서 Enter를 누르지 말고 기다린다.

```text
Relay and bridge are ready. Start the UPC and LAB recorders now.
```

### 4-3. UPC-B — UPC recorder 시작

UPC-A와 같은 `COND`를 사용한다.

```bash
cd "$HOME/rby1_ros2_ws/src/rby1-project"

COND=baseline

bash record_trial.sh \
  --role upc \
  --run-id "replay_${COND}" \
  --domain 10
```

시작 출력에 다음 값이 보여야 한다.

```text
[record] role=upc ROS_DOMAIN_ID=10
```

### 4-4. LAB-B — LAB recorder 시작

LAB-A 및 UPC와 같은 `COND`를 사용한다.

```bash
cd /workspaces/isaac_ros-dev/src/rby1-project
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=85

COND=baseline

bash rby1_vslam/scripts/capture_vslam_lab.sh \
  --run-id "replay_${COND}"
```

### 4-5. 재생 및 종료

1. UPC-B와 LAB-B가 모두 recording 중인지 확인한다.
2. UPC-A의 대기 프롬프트에서 Enter를 누른다.
3. bag 전체가 한 번 재생될 때까지 아무 조작도 하지 않는다.
4. UPC-A에 `Playback finished`가 나오면 2초 기다린다.
5. UPC-B를 `Ctrl+C`로 종료한다.
6. LAB-B를 `Ctrl+C`로 종료한다.
7. LAB-A cuVSLAM을 `Ctrl+C`로 완전히 종료한다.

`fixed_stereo` 계열에서는 재생이 끝날 때 relay가 다음 두 가지를 자동 검증한다.

- `returned_stationary` 마커를 받았는가
- 마커 뒤 coherent left/right/CameraInfo tuple을 실제로 고정했는가

검증 실패 시 UPC-A가 `replay validation failed`로 종료된다. 이 run은 결과에 포함하지
말고 source bag의 마커를 점검한다.

---

## 5. 네 조건 실행표

각 run마다 4장을 처음부터 반복하고, LAB cuVSLAM을 반드시 새로 시작한다.

| Run | UPC/LAB `COND` | LAB IMU |
|---:|---|---|
| 1 | `baseline` | `true` |
| 2 | `no_imu` | `false` |
| 3 | `fixed_stereo` | `true` |
| 4 | `fixed_stereo_no_imu` | `false` |

복사용 condition 목록:

```bash
COND=baseline
COND=no_imu
COND=fixed_stereo
COND=fixed_stereo_no_imu
```

한 run에서 UPC-A, UPC-B, LAB-B의 `COND`는 반드시 같아야 한다.

---

## 6. 각 run 직후 빠른 검증

UPC 결과:

```bash
LATEST_UPC="$(ls -td "$HOME"/rby1_trials/replay_*_upc_* | head -n 1)"

grep -E 'run_id|role|ROS_DOMAIN_ID' "$LATEST_UPC/environment.txt"
sed -n '1,80p' "$LATEST_UPC/timing_report.txt"
```

`ROS_DOMAIN_ID=10`이어야 하고 다음 토픽 count가 0이 아니어야 한다.

```text
/rby1/vslam/bridge_status
/rby1/vslam/camera_odometry
/rby1/vslam/camera_slam_odometry
```

LAB 결과:

```bash
LATEST_LAB="$(ls -td /workspaces/isaac_ros-dev/rby1_timing/replay_*_lab_* | head -n 1)"

grep -E 'run_id|role|ROS_DOMAIN_ID' "$LATEST_LAB/environment.txt"
sed -n '1,100p' "$LATEST_LAB/timing_report.txt"
```

LAB 결과는 `ROS_DOMAIN_ID=85`여야 한다.

---

## 7. 최종 그래프와 분석 항목

회전 구간과 `returned_stationary` 이후 마지막 10초를 분리해서 분석한다.

### 두 바퀴 회전 구간

- 누적 yaw와 720° 대비 오차
- 회전 종료 시 최종 위치 및 yaw 오차
- tracking loss와 `vo_state`
- tracking pose와 SLAM pose 차이
- loop/map correction 발생 시점과 크기

3번은 이 구간까지 1번과 같은 센서 입력이고, 4번은 이 구간까지 2번과 같은 센서
입력이다. 회전 구간 결과가 쌍 안에서 크게 다르면 재생 또는 초기화 조건부터 확인한다.

### 마지막 정지 10초

- pose 중심 이동량
- frame-to-frame translation/yaw jitter p95 및 max
- 10초 저주파 wander
- 고정 시작 직후 pose discontinuity
- image rate와 frame gap
- image-to-pose latency, TCP RTT, GPU 사용률

결과 폴더 이름은 아래 prefix를 유지한다.

```text
replay_baseline
replay_no_imu
replay_fixed_stereo
replay_fixed_stereo_no_imu
```

## 8. 실험 무효 조건

- 네 조건에 서로 다른 source bag을 사용함
- 조건 사이 LAB cuVSLAM/map을 재사용함
- playback 중 live D435 publisher가 함께 켜짐
- 두 바퀴 회전 완료 전에 `returned_stationary` Enter를 누름
- 최종 정지 10초 동안 로봇, 카메라 또는 주변 물체가 움직임
- 1·3번을 `enable_imu=false`로 실행하거나 2·4번을 `true`로 실행함
- UPC/LAB recorder의 condition 이름이 서로 다름

재생 속도는 모든 조건에서 기본값 `1.0`을 유지한다.
