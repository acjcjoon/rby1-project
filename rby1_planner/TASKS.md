# Planner Task 목록

`rby1_planner/rby1_planner/task.py` 의 `build_tasks()` 에 등록되어 Scenario UI 에 보이는 Task 목록.
(2026-11-18 기준. 장비 위치/태그/측정값은 `config/settings.yaml` 의 `stations`, 자세 관절값과 속도는 `task.py` 가 원본이다.)
Task 끼리의 포함 관계도 / 실행 순서는 [docs/task_graph.md](docs/task_graph.md) (자동 생성: `python3 scripts/gen_task_graph.py`).

카메라 이름: `d405_right` (오른손 손목), `d405_left` (왼손 손목), `d435` (머리).
손 배정: 문/스위치는 오른손, 물체 운반(집기/놓기)은 왼손 고정.

## 장비 (settings.yaml `stations`)

| 장비 | kind | 지점 (공통 좌표계, 허리 방향) | 태그 |
|---|---|---|---|
| `LH1` | plain | place [0.4, 0.0] front | 놓기/집기 tag_3 |
| `LH2` | plain | place [0.4, 0.81] right | 놓기/집기 tag_0 |
| `ZS` | door_switch | work [0, 0] left (문/스위치 작업 위치), via [-0.75, 0] left (경유), place [-0.75, 0.70] front | 놓기/집기 tag_2, 문 열기 tag_6, 문 닫기 tag_7, 스위치 tag_5 |
| `MPR` / `INC` / `WASTE` | plain | 자리표시자 (`enabled: false`) | 없음 |

- 지점 참조: `'LH1'` = LH1 의 place 지점, `'ZS.work'` 처럼 `장비.지점`.
- 태그 ID 는 장비 전체에서 한 번만 쓸 수 있다 (겹치면 Reload 오류).
- `kind` 에 따라 집기/놓기 순서가 정해진다 (아래 4).

## 1. 자세

| Task | 움직이는 부위 | 내용 |
|---|---|---|
| `ready_pose` | 토르소 + 양팔 | 기본 대기 자세 |
| `object_gripping_initial_pose` | 양팔 | 물체 파지 준비 자세 (허리 그대로) |
| `object_gripping_initial_pose(front)` / `(left)` / `(right)` | 토르소 + 양팔 | 허리(yaw 0° / +90° / -90°)와 양팔 파지 준비 자세를 함께 맞춘다 |
| `search_pose` | 오른팔 | 문 손잡이 탐색 자세 |
| `search_switch_pose` | 토르소 + 양팔 | 스위치 탐색 자세 |
| `door_handle_grasp_pose` | 토르소 + 양팔 | 손잡이 파지 관절값 |

## 2. 기본 동작

| Task | 카메라 / 태그 | 내용 |
|---|---|---|
| `align_base_to_door(open)` / `(close)` | d435 / ZS `door.open.tag`(tag_6) / `door.close.tag`(tag_7) | 태그가 측정해 둔 위치(`base_align_xy`)에 보이도록 base 이동 (yaw 미보정) |

## 3. 장비 동작 (ZS, 오른손)

문 열기/닫기와 스위치 누르기는 UI 에 등록하지 않고 `place_at` / `pick_at` 안에서만 쓴다 (아래 helper 표).

## 4. 장비 단위 (왼손 집기/놓기)

출입 지점에서 시작해 출입 지점에서 끝난다 (plain = place 지점, door_switch = work 지점).

| Task | 내용 |
|---|---|
| `place_at_station(ZS)` | 머리 32° → 문 열기 → 손 옮기기 → 스위치 → 손 빼기 → 머리 2° → work → via → place 이동 → 머리 47°(`points.place.head_pitch_deg`)로 숙이고 tag_2 에 놓기 → 머리 2° → place → via → (이동 전 자세) → work → 머리 32° → 닫기 손잡이 기준 base 정렬 → 스위치 → 문 닫기 → 손잡이 놓고 손 빼기 (-y 0.10, 실기 확인 전) |
| `pick_at_station(ZS)` | 위와 같고 놓기 대신 집기 |

plain 장비(LH1, LH2)는 태그로 base 정렬 후 집기/놓기만 한다.

## 5. 운반 (UI 미등록, 데모와 스케줄러에서 사용)

- `transfer(A, B)`: 시작 자세 → A 에서 집기 → B 로 이동 → B 에 놓기. **로봇이 A 출입 지점에 있다고 본다.**
  다른 곳에 있으면 `transfer(A, B, start=C)` 로 C → A 이동을 먼저 넣는다. `start` 없이 다른 곳에서 실행하면
  A 태그가 안 보이거나(5 s 후 실패) 0.8 m 넘게 떨어져 있어(거부) 정렬 단계에서 멈춘다.
- 스케줄러 Action(`transfer_action_server.py`)은 마지막 도착 장비를 기억해 `start` 로 넘긴다
  (처음은 settings.yaml `current_location`, 운반 실패 시 위치 모름 → 다음은 source 에 있다고 가정).

## 6. 데모

| Task | 팔 | 내용 |
|---|---|---|
| `transfer_demo(LH1->ZS, LH1->LH2)` | 왼팔 (+ 오른팔 문/스위치) | LH1 에서 시작. `transfer(LH1,ZS)` → `transfer(LH1,LH2, start=ZS)` (ZS 에서 LH1 로 이동부터) |
| `impedance_demo_press` | 오른팔 | 관절 임피던스로 base -z 3 cm 접촉 후 5 cm 상승 |
| `impedance_demo_joint` | 오른팔 | 팔꿈치를 관절 임피던스로 10° 굽혔다가 위치제어로 복귀 |
| `wave_both_arms` | 양팔 + 토르소 | 그리퍼 닫고 양팔 들어 좌우로 3회 흔들기. 빈손 전용 |

## 7. 테스트

| Task | 내용 |
|---|---|
| `test0` | 허리 유지. D435 로 tag_3 보고 base 정렬 → 왼손 파지 |
| `test1` | 왼손만. base·허리 고정. tag_3 ↔ tag_0 픽앤플레이스 10 회 반복 |
| `test2` | LH1 → LH2 → LH1 왕복 (`transfer` 두 번) |

## 8. VSLAM

| Task | 내용 |
|---|---|
| `offline_map_creation` | 운영 전 오프라인 맵 생성. 머리 2°(`head_pitch_deg.move`) → cuVSLAM mapping 시작·tracking 대기 → 출발 지점 보정 → **작업자가 Base 탭으로 지점을 돌며 VSLAM 탭에서 `Stop + correct` / `Save Point`** → `Finish mapping` → 지도를 `map_<날짜>_<시각>` 으로 저장. 끝나도 VSLAM 은 켜 둔다. 조그 전에 상단 Stream 을 켠다 |

## 등록되지 않은 helper

| 함수 | 내용 |
|---|---|
| `go_to(A, B)` | 장비 출입 지점 사이 이동 (`turn_and_move`) |
| `turn_and_move(a, b)` | 지점 참조 사이 base 직선 이동. 도착 지점 허리 방향이 다르면 먼저 맞춘다. 머리는 이동 전 `head_pitch_deg.move`(2°)로 맞추고 도착 후에는 건드리지 않는다 (뒤따르는 장비 작업이 지점 각도로 맞춤) |
| `head_pitch_pose(deg)` | 머리 pitch 절대각으로 맞춤 (+ = 숙임, yaw 0). 각도는 settings.yaml `head_pitch_deg.move` / `points.<지점>.head_pitch_deg` |
| `align_base_to_tag(tag, arm=...)` | d435 로 태그를 보고 base 정렬 (목표 = `arm` 손목 카메라 아래) + 1 s 대기. **`arm` 필수**. 뒤의 `*_in_place` 에 같은 태그·같은 `arm` 을 넘길 것 |
| `move_hand_to_tag(tag, approach, contact, motion, arm=...)` | 손목 카메라로 태그를 보고 TCP 를 접근 → 접촉 지점으로 이동 (태그별 YAML 보정 포함, yaw 만 따름). `move_hand_to_door_handle` 의 태그 버전 |
| `pick_up_object_in_place` / `put_down_object_in_place` | base 이동 없이 현재 자세에서 손목 카메라로 파지/배치 + 7.5 cm 상승 |
| `open_door(st)` | 그리퍼 열기 → `search_pose` → `align_base_to_door(open)` → 손잡이로 손 이동 → 잡기 → `door_open_stream` |
| `close_door(st)` | (스위치를 누른 뒤) 닫기 손잡이 쪽으로 손 이동 → 잡기 → `door_close_stream`. base 는 `align_base_to_door(close)` 로 맞춰져 있어야 한다 |
| `push_switch(st)` | `search_switch_pose` → 그리퍼 닫기 → D405 로 스위치 태그 접근 → base 좌표 직선 누름/후퇴 → `search_switch_pose` |
| `move_hand_to_door_handle(st, open/close)` | d435 로 문 태그를 보고 오른손 TCP 를 `ee_offset_tag` (태그 축) 위치로 이동 |
| `door_open_stream` / `door_close_stream` | 오른손 원호 스트림으로 문 열기/닫기 (경첩 기하 `work.door.hinge_to_handle_wall` (벽 좌표), `swing_deg`) |
| `move_hand_from_door_to_switch` | 문을 연 뒤 손잡이를 놓고 오른손을 스위치 쪽으로 옮긴다 |
| `pose_before_base_movement` | ZS 복귀 시 work 로 들어가기 전 자세 |

## 남은 일

- LH ↔ ZS 운반은 공통 좌표계 값(LH1 [0.4, 0.0], LH2 [0.4, 0.81])으로 구성만 되어 있고 실기 확인 전이다.
- 문/스위치 사이 손 옮기기 이동량(`move_hand_from_door_to_switch`, `close_door`, 손 빼기 0.25 m)은 ZS 측정값이고 코드 상수다. 벽 좌표로 적혀 있어 배치가 바뀌면 `work.wall_yaw_deg` 만 고치면 된다.
