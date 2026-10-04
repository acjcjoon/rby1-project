# 일반 실행과 시험 녹화

모든 일반 런처는 **UPC에서** 실행한다. 네 개 중 하나만 켠다.

| 모드 | 웹 (Windows 브라우저) | UPC 화면 (Qt) |
|---|---|---|
| 베이스 수동 이동 | `bash run_mobile_base_web.sh` | `bash run_mobile_base_upc.sh` |
| TCP VSLAM + Nav2 | `bash run_vslam_web.sh lab_host:=<LAB-IP>` | `bash run_vslam_upc.sh lab_host:=<LAB-IP>` |

베이스 모드는 driver/control/UI만 실행한다. 카메라, TCP, VSLAM, Nav2는 실행하지 않는다.
VSLAM 모드는 기존 physical launch로 driver/control/D435i/TCP/위치 추정/Nav2/UI를 실행한다.
UPC VSLAM UI는 기존 Qt 웨이포인트 화면과 RViz다. UPC 베이스 UI는 수동 이동 전용 Qt다.
모든 UI는 전원/서보/제어/stream을 사용자가 켠 뒤 이동하며 자동으로 이동하지 않는다.
전원·서보 ON은 기존 제어와 동일하게 `all` 대상이다. UI STOP은 하드웨어 비상정지가 아니다.

## 설치와 공통 설정

UPC workspace에서 의존성을 준비하고 빌드한다 (로컬 Qt는 `python3-pyqt5` 필요).

```bash
source /opt/ros/humble/setup.bash
cd ~/rby1_ros2_ws
colcon build --symlink-install --packages-up-to rby1_web rby1_description
source install/setup.bash
cd src/rby1-project
```

기본 workspace는 `~/rby1_ros2_ws`, 기본 UPC domain은 현재 환경 또는 `0`이다.
다른 설치는 `RBY1_WORKSPACE_SETUP`, `RBY1_ROS_SETUP`으로 지정한다.
도메인은 런처와 녹화 터미널 양쪽에 같은 값으로 설정한다.

```bash
export ROS_DOMAIN_ID=0
bash run_mobile_base_web.sh robot_address:=192.168.30.1:50051 web_port:=8080
```

웹 주소는 `http://<UPC-IP>:8080`이다. Qt는 UPC 데스크톱 세션에서 실행한다.
추가 인자는 기존 ROS `name:=value` 형식이며 UI 선택은 파일 이름으로 고정된다.
VSLAM의 `use_scan:=false` 기본값에는 장애물 회피가 없다. LiDAR가 준비된 경우
`use_scan:=true`를 사용하고, 정렬된 occupancy map은 `occupancy_map:=/절대경로/map.yaml`로 지정한다.

## LAB 준비 (VSLAM 모드에서만)

LAB의 Isaac ROS 컨테이너에서 기존 보조 런처를 실행한다.

```bash
bash scripts/run_vslam_lab.sh --mode mapping --domain 85 --no-rviz
```

위 경로는 컨테이너의 `rby1_vslam` 패키지 폴더 기준이다. 기존 localization 모드도
`--mode localization --map-path /절대경로/지도`로 선택 가능하다.
UPC/LAB의 도메인은 서로 같을 필요가 없고 데이터는 TCP로 전달된다.

VSLAM 일반 런처와 LAB/UPC/mapping 보조 런처는 모두 **IMU 사용이 기본값**이다.
UPC는 D435i의 gyro/accel을 통합한 `/d435/d435/imu`를 TCP로 전달하고,
LAB cuVSLAM은 `tracking_mode=1`로 영상과 IMU를 함께 사용한다.
UPC VSLAM 런처는 Jetson에서 IMU를 사용할 수 있도록 기본적으로
`~/librealsense-rsusb-2.58.4/build-rsusb/Release`의 RSUSB librealsense를 강제한다.
다른 위치를 사용하려면 `RBY1_RSUSB_LIB_DIR`에 `Release` 디렉터리를 지정한다.
진단용으로 IMU를 끄려면 일반 UPC 런처에 `enable_imu:=false`,
LAB 보조 런처에 `--enable-imu false`를 함께 지정한다.
베이스 이동 전용 모드는 카메라/IMU/VSLAM을 시작하지 않는다.

## 시작부터 Point2 정지까지 한 번의 녹화

`record_trial.sh`는 런처와 별도로 실행한다. UPC와 LAB의 시험 이름만 일치시킨다.
LAB 컨테이너에는 이 스크립트뿐 아니라 `rby1_vslam/scripts/timing_observer.py`와
`analyze_vslam_timing.py`도 함께 있어야 한다. 전체 저장소 경로에서 실행하거나,
`record_trial.sh`를 LAB의 `rby1_vslam` 패키지 루트에 두면 `scripts/`를 자동으로 찾는다.

1. UPC 녹화 터미널에서 **먼저** 실행한다.
   `--domain`은 UPC 실행 스크립트가 시작할 때 출력한 값과 반드시 같게 한다.
   아래 `10`은 `run_vslam_mapping_upc.sh` 기본값 예시이며, 전체 web 런처를 domain 0으로
   실행했다면 `0`을 사용한다.

   ```bash
   bash record_trial.sh --role upc --domain 10 --run-id point1_point2_01
   ```

2. LAB 컨테이너 녹화 터미널에서 실행한다. 출력 경로는 지속 보존되는 마운트 경로로 지정한다.

   ```bash
   bash record_trial.sh --role lab --domain 85 --run-id point1_point2_01 \
     --output /workspaces/isaac_ros-dev/trials
   ```

3. LAB VSLAM과 UPC `run_vslam_web.sh lab_host:=<LAB-IP>`를 별도 터미널에서 실행한다.
   이미 LAB이 실행 중이면 중복 실행하지 않는다. 각 `recorder.log`에서 구독 시작을 확인한다.
4. 웹에서 초기 위치 Point1 저장 → 수동 이동 후 Point2 저장 → Point1 근처로 복귀 → Point2 이동.
5. 멈춘 뒤 웹을 전면에 유지하고 10~20초 더 기록한다. 웹 포커스 상실/연결 끊김은 STOP·취소를 유발할 수 있다.
6. 웹 STOP 후 **녹화 터미널 각각에서 Ctrl+C**. 저장 완료까지 기다린다. 녹화 종료는 로봇을 멈추지 않는다.
   노드 설정을 수집할 수 있도록 전체 스택은 녹화 종료가 끝날 때까지 유지한다.

기본 저장 경로는 `~/rby1_trials/<시험>_<역할>_<시간>_<고유값>/`이다.

- `bag/`: 기본값은 rosout, TF, diagnostics, bridge/pose/status/action/control 등 저용량 디버깅 필수 토픽. 원본 영상과 VSLAM point cloud는 제외
- `recorder.log`, `bag_info.txt`: 구독 경고, 기록된 토픽/메시지 수, bag 확인 결과
- `events.jsonl`, `observer.log`: 카메라→LAB→cuVSLAM pose→UPC 반환 pose의 stamp/rate/gap과 VSLAM·Nav2 상태
- `timing_report.txt`, `timing_report.json`, `analysis.log`: Ctrl+C 종료 후 자동 생성되는 해당 PC 병목 요약
- `environment.txt`, `system_samples.txt`: domain/배포판/시계/디스크/시스템 load
- `nodes_start/end.txt`, `topics_start/end.txt`, `params_start/end/`: 노드/토픽/파라미터
- `waypoints_start/end.yaml`: UPC 기본 Point YAML의 복사본. 사용자 지정 파일은 `--waypoints /경로/points.yaml` 사용

원본 영상까지 반드시 필요할 때만 `--full-bag`을 붙인다. 이 옵션은 LAB Python bridge의 영상 publish를 지연시켜 500 ms stale 판정을 유발할 수 있으므로 정상 통신을 먼저 확인한 뒤 짧게 사용한다.
QoS/구독 실패 메시지가 있으면 실제 토픽 설정에 맞춰 보완해야 한다.
rosbag 녹화는 서비스 요청·응답 전체 또는 비 ROS 웹 이벤트를 보장하지 않는다.
cuVSLAM 지도 저장도 별도이며, LAB mapping 종료 전에 기존 `map_tool save`로 저장한다.
실제 실행 옵션과 시험 증상/시각은 시험 폴더에 메모한다.

양쪽 결과를 한 PC에 모은 뒤 다음처럼 합치면 전송 전·후 구간까지 자동 비교한다.

```bash
python3 rby1_vslam/scripts/analyze_vslam_timing.py \
  ~/rby1_trials/point1_point2_01_upc_YYYYMMDD_HHMMSS_XXXXXX \
  ~/rby1_trials/point1_point2_01_lab_YYYYMMDD_HHMMSS_XXXXXX \
  --output-dir ~/rby1_trials/point1_point2_01_combined
```

## 기존 보조 스크립트

`run_planner_ui.sh`는 팔·그리퍼·시나리오를 포함하는 별도 전체 planner 도구다.
일반 베이스 런처 대신 사용하지 않는다.
`rby1_vslam/scripts/run_vslam_upc.sh`는 센서/TCP만 켜는 보조 도구이며,
루트의 동명 런처는 **Nav2/UI 포함 전체 UPC 실행**이다.
`run_vslam_mapping_upc.sh`는 Nav2 없는 수동 mapping 전용 도구 (기본 domain 10)다.
기존 `capture_vslam_*`는 영상 원본 없이 지연을 계측하는 경량 도구로 유지한다.
