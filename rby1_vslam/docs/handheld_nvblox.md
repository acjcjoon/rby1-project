# 분리한 D435i로 RGB 3D 재구성

카메라를 손으로 움직이며 RGB mesh와 카메라 궤적을 확인하는 독립 실행 모드다.
[NVIDIA Isaac ROS 3.2 RealSense 예제](https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/scene_reconstruction/nvblox/tutorials/tutorial_realsense.html)의
연속 stereo VO + native depth/RGB 구성을 현재 설치 환경에 맞게 사용한다.
최신 문서의 다른 ROS/Isaac 버전으로 업그레이드할 필요는 없다.

## 실행

기존 카메라 프로세스는 먼저 종료한다. 같은 D435i를 두 프로세스에서 열지 않는다.
이 모드에는 베이스 driver, robot/head TF, Nav2, station correction이 필요 없다.
기본 ROS domain은 **11**이며 기존 로봇 실험 domain 10과 분리된다.
각 터미널에서 다음 디렉터리로 이동한다.

```bash
cd /home/nvidia/rby1_ros2_ws/src/rby1-project
```

터미널 1 — 카메라:

```bash
RBY1_D435_SERIAL=244222075961 bash rby1_vslam/run_handheld_nvblox.sh camera
```

터미널 2 — splitter, 카메라 VO, 컬러 nvblox:

```bash
bash rby1_vslam/run_handheld_nvblox.sh mapping
```

터미널 3 — RGB mesh 시각화:

```bash
bash rby1_vslam/run_handheld_nvblox.sh rviz
```

세 실행 모두 domain을 바꾸려면 같은 `RBY1_HANDHELD_ROS_DOMAIN_ID`를 설정한다.
카메라의 32 MiB DDS 전송 설정과 RViz shader fallback은 자동 적용된다.
설치나 sudo 명령은 필요하지 않다. 종료는 각 터미널에서 Ctrl-C로 한다.

## 화면과 실험 순서

- `RGB Camera`: 실제 색상 영상.
- `RGB Mesh`: depth로 만든 표면에 RGB 색상을 적분한 3D mesh.
- `Camera Axes`, `Camera Path`: 연속 VO가 추정한 카메라 위치·방향과 이동 궤적.
- `Distance Slice (optional)`: 필요할 때만 켜는 ESDF 거리 표시. 기본은 꺼져 있다.

처음 몇 초간 카메라를 정지시켜 추적과 표면 생성을 확인한다. 밝은 환경에서
무늬가 있는 정적인 대상과 벽을 약 1~3 m 거리에서 천천히 관측한다.
카메라를 좌우로 20~30 cm 이동하고 작은 각도로 회전하며 같은 표면이 겹치지 않고
유지되는지 확인한다. RGB 영상과 mesh의 물체 경계도 비교한다.

mesh가 갑자기 이동하거나 이중 표면이 생기면 멈추고 VO 추적 상태와 시간 간격을
확인한다. 실패한 지도 위에 계속 누적하지 않고 `mapping`을 다시 시작해 새 지도로
비교한다. USB를 분리했다면 카메라와 mapping을 모두 다시 시작한다.

이 모드의 원점은 VO 초기화 시점의 카메라 위치다. 바닥 높이·중력 방향이 보정된
좌표계가 아니므로 ESDF slice는 바닥 기준 costmap으로 해석하지 않는다.
RGB 시야 밖이거나 아직 색상 관측이 없는 표면은 회색일 수 있다.

## 설정과 검증 범위

| 항목 | handheld 기본값 |
|---|---|
| native IR/depth | 848×480 @ 90 Hz, emitter 교대로 각 45 Hz 선택 |
| RGB 입력 | RGB8 640×480 @ 30 Hz |
| voxel | 3 cm |
| 적분 최대 거리 | 4 m |
| depth / color 적분 목표 | 45 / 10 Hz |
| mesh / ESDF 갱신 목표 | 5 / 10 Hz |
| 지도 유지 범위 | 카메라 주위 반경 8 m |
| 카메라 포즈 | cuVSLAM stereo, 6DoF, ground constraint 없음 |

TF는 cuVSLAM의 `vslam_odom → d435_link`와 RealSense의 공장 보정 optical TF로
구성한다. 기존 wheel odometry나 head 장착 보정을 사용하지 않는다.
depth와 RGB는 각 영상의 촬영 시각 TF를 기다려 적분하며 원본 timestamp를 유지한다.
RGB/depth는 각각 자신의 CameraInfo와 optical frame을 사용하고 depth alignment를
강제로 적용하지 않는다. 실제 카메라 조회에서 RGB와 depth의 왜곡 계수는 모두 0이었다.

공식 3.2 예제와 같이 연속 VO를 사용한다. 이 모드에서는 loop closure나 과거 TSDF
변형을 수행하지 않으므로 긴 이동에서는 VO drift가 누적될 수 있다.
3 cm는 voxel 크기이며 절대 위치 정확도 보장이 아니다. RGB는 표면 색상을 추가하고,
형상 정확도는 depth 품질과 포즈·시간 정합에 달려 있다. mesh는 vertex color 방식이며
사진 텍스처를 붙이는 고해상도 스캔과 표현 방식이 다르다.

부하 비교용 5 cm voxel 실행:

```bash
bash rby1_vslam/run_handheld_nvblox.sh mapping voxel_size:=0.05
```

색상 영상 해상도는 카메라 실행의 추가 인자로 변경할 수 있다. 변경 후 처리율과
색상 경계 정합을 다시 확인한다.

설정 파일은 `config/handheld_nvblox.yaml`, 실행은
`launch/handheld_nvblox.launch.py`, 화면은 `config/handheld_nvblox.rviz`다.
기존 로봇용 localization 및 nvblox 설정은 별도 실행 경로로 유지한다.

## 2026-10-26 검증 기록

- 실제 D435i에서 IR/depth 848×480@90, RGB8 640×480@30 동시 입력 확인.
- 실제 mesh 메시지에서 vertex와 color 배열 각각 최대 12,482개,
  표본 색상 16,692개 중 비회색 색상 6,179개를 확인해 RGB 적분을 검증했다.
- 동적 카메라 TF의 유일한 parent는 `vslam_odom`이었다.
- 대용량 mesh를 함께 Python으로 분석할 때는 계측 구독에 누락이 있었으므로
  VO 성능은 별도의 경량 계측으로 확인했다. 마지막 구간의 VO는 44.93 Hz,
  pose 826개, 최대 timestamp 간격 22.27 ms, 40 ms 초과 간격 0회였다.
- 마지막 실행에서 nvblox depth 약 45 Hz, color 약 10 Hz,
  mesh 약 4.8~5 Hz, ESDF 약 9.3 Hz를 관측했고 RViz 종료 오류는 없었다.
- 사용자가 카메라를 일부 움직였으므로 이 구간은 정지 정확도 시험으로 분류하지
  않는다. 기준 물체·외부 ground truth에 대한 이동 정확도는 아직 측정하지 않았다.
- USB/카메라 분리 작업과 겹친 실행은 IR 중단으로 평가에서 제외했다.
  마지막 실행 시작 시에도 `Left MIPI error` 알림은 있었지만, 이후 관측 구간의
  metadata·VO는 연속적으로 수신됐다. 이 기록은 하드웨어 오류가 영구 해결됐다는
  의미가 아니며 같은 알림과 함께 IR/VO가 중단되면 연결 상태를 확인해야 한다.

측정 원본은 `/tmp/rby1-handheld-rgb-check/measurements.json`과
`/tmp/rby1-handheld-rgb-stable-check/measurements.json`에 있다.
