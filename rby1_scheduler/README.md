# RB-Y1 ROS Scheduler

ROS 2 `ament_python` 패키지다. 고정 환경과 워크플로우는 `config/lab.yaml`, 현재 Plate와 작업 상태는 `mock_server/server_input.json`에서 읽는다.

## 입력 분리

- `lab.yaml`: 장비 작업/Plate 용량, 공정 시간, 구간별 이송 시간, S/T 워크플로우, 선행 조건, 기본 우선순위
- `server_input.json`: 배치, Plate 위치·바코드·몰비·완료 작업, 현재 작업 큐와 상태
- `scheduler_output.json`: 라이브 스케줄러 상태 출력
- `scheduler_events.jsonl`: 라이브 실행 이벤트

JSON 작업 큐에 경로·우선순위·선행 조건이 없으면 `lab.yaml`의 같은 단계 ID에서 자동으로 채운다.

## PyQt UI

- 스케줄링 탭
  - `Sim`: 현재 JSON 상태를 복사하고 `lab.yaml` 기준 가상 시간 시뮬레이션을 실행한다. Planner 호출과 서버 파일 변경은 하지 않는다.
  - `Play`: 정지 상태인 라이브 스케줄러를 실행한다.
  - `Pause`: 다음 라이브 작업 선정을 정지한다. 이미 실행 중인 작업은 취소하지 않는다.
  - 라이브 작업 큐, 장비 점유, 배치, 시뮬레이션 타임라인, 이벤트를 표시한다.
- 플레이트 정보 탭: 바코드, 배치 몰비, 위치, 현재/완료 작업
- 통신 테스트 탭: 수동 Pick/Place, Planner 상태와 마지막 결과

현재 `Play` 경로의 Planner는 약 4초 후 성공하는 `MockPlannerClient`다. 실제 로봇 연동 시 같은 `PlannerClient` 경계를 ROS Action client로 교체한다.

## 실행

패키지를 ROS 2 작업공간에 복사한 뒤 실행한다.

```bash
colcon build --packages-select rby1_scheduler
source install/setup.bash
ros2 launch rby1_scheduler scheduler.launch.py
```

설정 경로를 바꿀 수 있다.

```bash
ros2 launch rby1_scheduler scheduler.launch.py \
  lab_config_path:=/data/lab.yaml \
  server_input_path:=/data/server_input.json \
  scheduler_output_path:=/data/scheduler_output.json \
  scheduler_events_path:=/data/scheduler_events.jsonl
```
