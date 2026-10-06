# RBY1 메카넘 근거리 Nav2 진단 및 MPPI 적용

적용일: 2026-10-06. 대상은 ROS 2 Humble, RBY1-M 메카넘 베이스, Theta* + MPPI
`Omni` 구성이다.

## 대상 왕복 구간

| 포인트 | x [m] | y [m] | yaw [rad] |
|---|---:|---:|---:|
| Point 1 | -0.108110 | -0.004857 | -0.003806 |
| Point 2 | 1.076058 | 0.049006 | -0.011037 |

두 점은 1.18539 m 떨어져 있고 직선 방위는 2.604°, 목표 yaw 차이는 0.414°다. 5 cm
costmap에서 약 23.7 cell이므로 같은 cell에 들어가는 초근접 목표가 아니다. Point 2에서
Point 1로 돌아올 때도 yaw가 거의 같아, 로봇이 180° 회전하는 대신 음의 `vx`로 후진하는
왕복 조건이다.

## 적용 결론

로컬 컨트롤러를 DWB에서 MPPI로 바꾸고 다음을 함께 적용했다.

1. 메카넘 횡이동을 쓰도록 Humble의 대소문자를 그대로 맞춘 `motion_model: "Omni"`를
   사용한다. `PreferForwardCritic`은 제외하고 `PathAngleCritic.forward_preference: false`로
   설정해 같은 yaw를 유지하는 복귀 후진을 허용한다.
2. controller 20 Hz와 `model_dt: 0.05`를 일치시켰다. 56 step은 2.8초 지평선이며,
   설정한 최대 합성 평면 속도에서 약 0.40 m를 본다. 5×5 m local costmap 안에 충분히
   들어오며 critic offset 2~3은 이 거리의 약 8개 5 cm path sample에 맞춘 값이다.
3. global/local costmap 해상도는 0.05 m로 유지한다. 1 cm로 줄이면 cell 수가 25배가
   되지만 다음 Theta* 끝점 문제는 해결을 보장하지 않는다. MPPI의 상태/goal score는
   연속 좌표에서 계산되므로 goal tolerance 때문에 costmap도 1 cm일 필요는 없다.
4. `PoseProgressChecker` 0.03 m/3°/30초와 `failure_tolerance: 1.0`을 적용했다. 의미 있는
   회전도 progress로 인정하고, 3°보다 작은 미세 보정에는 30초를 주며, 일시적인 optimizer
   재시도만으로 status 6이 나오는 것을 줄인다.
5. `StoppedGoalChecker`와 `stateful: false`를 사용해 위치 1 cm, yaw 1°, 평면 속도
   0.02 m/s, 회전 속도 0.02 rad/s를 **동시에** 만족해야 성공한다.

남은 구조적 제한은 global planner 끝점이다. Humble Theta*는 시작점과 목표점을 costmap
cell로 변환한다. 두 점이 같은 cell이면 현재 위치 하나만 담긴 path를 반환하고 목표 yaw만
적용한다. 일반 경로의 보간도 실제
   요청 goal을 마지막 점으로 항상 보존하지 않는다. 따라서 path 끝이 요청점에서 수 cm
   떨어질 수 있다. [Humble Theta* 구현](https://github.com/ros-navigation/navigation2/blob/humble/nav2_theta_star_planner/src/theta_star_planner.cpp)
MPPI도 Theta*가 path에서 잃어버린 실제 goal 좌표를 복구할 수 없다. 로그에서
`requested goal -> /plan end` 오차를 확인한다. 오차가 1 cm보다 크면 planner 쪽을 먼저
고쳐야 한다. action status 6은 Nav2 고유 상태가 아니라 `ABORTED`이며 gate 취소의 일반적인
상태 5 `CANCELED`와 구분한다. [ROS 2 Humble GoalStatus](https://github.com/ros2/rcl_interfaces/blob/humble/action_msgs/msg/GoalStatus.msg)

## 베이스 형상과 메카넘 반영

사용자가 제공한 외곽 치수에 따라 전후 길이 0.695 m, 좌우 폭 0.600 m의 직사각형을
두 costmap에 적용했다.

```yaml
footprint: "[[0.3475, 0.3000], [0.3475, -0.3000], [-0.3475, -0.3000], [-0.3475, 0.3000]]"
footprint_padding: 0.05
```

이는 `base` 원점이 외곽 직사각형의 기하 중심이고 x가 전방, y가 좌측이라는 가정이다.
실제 원점이 치우쳐 있으면 앞/뒤 길이를 따로 재서 vertex를 보정해야 한다. 0.05 m
padding 때문에 충돌 검사는 실측 외곽보다 보수적이다. 팔을 펼친 외곽은 이 polygon에
포함되지 않는다.

현재 MPPI는 `vx`, `vy`, `wz`를 모두 최적화하고 `CostCritic.consider_footprint: true`로
이 padded rectangle 전체를 충돌 검사에 사용한다.

## RPP와 MPPI 검토

| 항목 | 이전 holonomic DWB | RPP (Humble) | 현재 MPPI (Humble) |
|---|---|---|---|
| 메카넘 횡속도 `vy` | 사용 | 출력하지 않음 | `Omni` model에서 사용 |
| 계산량 | 중간 | 가장 낮음 | 가장 높음 |
| 현재 문제의 Theta* endpoint 수정 | 불가 | 불가 | 불가 |
| 현재 역할 | 이전 비교 기준 | 이번 RBY1에는 비권장 | 적용된 실물 시험 대상 |

Humble RPP 구현의 명령은 `linear.x`와 `angular.z` 중심이라 메카넘의 횡이동 이점을 쓰지
못한다. [Humble RPP 소스](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)

MPPI는 `motion_model: "Omni"`와 `vy_max`, `vy_std`를 제공하므로 두 후보 중 RBY1에 더
맞다. 초기값은 full-footprint 검사 부하를 고려해 1000 batch로 잡았다. 후보 trajectory
시각화는 끄고 UPC에서 controller 주기 p99를 확인한 뒤 여유가 있을 때만 batch를 늘린다.
[Humble MPPI 패키지](https://github.com/ros-navigation/navigation2/tree/humble/nav2_mppi_controller)

## 실물 검증 순서

1. 아래 진단을 켠 뒤 Point 1 → Point 2 → Point 1 한 왕복을 수행한다.
2. 각 leg의 `/plan` endpoint 오차와 status 6 직전 로그를 확인한다.
3. endpoint가 1 cm보다 틀리면 다음 중 하나를 적용한다.
   - 기준 실험: NavFn `tolerance: 0.0`으로 실제 요청 goal을 보존하는지 비교한다.
   - Theta* 유지: planner 결과 끝에 실제 요청 pose를 충돌 검사 후 추가하는 wrapper/plugin을
     만든다. 목표 yaw도 요청값을 그대로 보존한다.
4. 정지 상태 localization 보정의 p95/max가 0.03 m/3°보다 작은지 확인하고 progress
   threshold를 실제 노이즈보다 크게 다시 정한다. 1 cm/1°보다 큰 정지 jitter가 있으면
   controller를 더 튜닝하기 전에 localization을 먼저 고친다.
5. 20 Hz missed-loop가 없고 p99 계산 시간이 40 ms 미만인지 확인한다. 여유가 확인된
   경우에만 `batch_size`를 1000에서 1500 또는 2000으로 올려 A/B한다.

`NavigateThroughPoses`를 사용하는 별도 클라이언트에서는 현재 BT의
`RemovePassedGoals radius="0.3"`도 30 cm 이내 중간점을 제거할 수 있다. 현재 Qt UI의
점 이동은 개별 `NavigateToPose`이므로 이 항목은 직접 원인이 아니다.

## 진단 실행

먼저 변경사항을 UPC에서 빌드한 뒤 Nav2 debug 로그를 켠다. 아래 `use_scan:=true`는
`/scan`이 실제 발행 중일 때만 사용한다. LiDAR를 쓰지 않은 기존 재현이라면
`use_scan:=false`로 동일 조건을 유지하되 장애물 감지/회피가 없음을 전제로 한다.

```bash
cd ~/rby1_ros2_ws
colcon build --symlink-install --packages-up-to rby1_control rby1_vslam rby1_description

cd ~/rby1_ros2_ws/src/rby1-project
bash run_vslam_upc.sh \
  lab_host:=<LAB-PC-IP> \
  use_scan:=true \
  nav2_log_level:=debug
```

별도 UPC 터미널에서 goal을 보내기 전에 수집기를 시작한다.

```bash
cd ~/rby1_ros2_ws/src/rby1-project
bash rby1_vslam/scripts/capture_navigation_debug.sh
```

`Nav2 ready parameter snapshot complete` 문구를 확인한 뒤 Point 1 → Point 2 → Point 1
한 왕복을 재현하고, UI에서 먼저 정지한 다음 상태 변화가 기록되도록
약 10초 기다린 뒤 수집기 터미널에서 `Ctrl+C`를 누른다. stack은 수집기가 끝날 때까지
켜 두어야 마지막 parameter dump가 성공한다. 수집기는 service/action/parameter를
변경하거나 이동 명령을 보내지 않는다.

기본 결과 디렉터리는 `~/rby1_debug/nav2_debug_upc_<시각>_<고유값>/`이다.

- `timing_report.txt`: status 6, child action, 관련 Nav2 로그, path endpoint 오차,
  `vslam_map -> odom` 보정 step의 p50/p95/p99/max, 1 cm/1° 초과 횟수와 wheel
  odometry 기준 정지 상태 jitter 통계
- `timing_report.json`: 위 결과와 correction/path 표본의 기계 판독용 데이터
- `params_ready/`, `params_end/`, `params_ready_to_end.diff`: Nav2가 실제 사용한 시작/종료
  파라미터와 실행 중 변화
- `events.jsonl`: parameter event, localization correction, action/path metadata 시계열
- `bag/`: TF, odometry, localization/gate 상태, 명령 속도 3단계, parent/child action,
  global/local path, scan, costmap, footprint
- `runtime_config_ready/`: 실행 중 설치본의 `navigation.yaml`과 BT XML 사본

MPPI의 `visualize`는 배포 설정에서 껐다. Humble의 `transformed_global_plan`은 수집하지만
1000×56 후보를 MarkerArray로 만드는 `trajectories` 시각화는 controller 부하와 timing을
바꾸므로 기본 진단에서 사용하지 않는다. 필요하면 짧은 전용 튜닝 실행에서만 켠다.

Nav2 parameter는 보통 launch 뒤 자동으로 계속 변하는 값이 아니다. 외부에서
`ros2 param set` 또는 parameter callback을 실행하지 않았다면 ready/end diff가 비어 있는
것이 정상이다. costmap, path, controller feedback처럼 매 cycle 변하는 값은 parameter가
아니며 bag과 `events.jsonl`에서 별도로 확인한다.
