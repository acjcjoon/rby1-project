# Planner Task 목록

`rby1_planner/rby1_planner/task.py` 의 `build_tasks()` 에 등록되어 Scenario UI 에 보이는 Task 26 개.
(2026-11-16 기준. 값/오프셋은 `task.py` 와 `config/settings.yaml` 이 원본이다.)

카메라 이름: `d405_right` (오른손 손목), `d405_left` (왼손 손목), `d435` (머리).
손 배정: 문/스위치 Task 는 오른손(`d405_right`), 물체 픽앤플레이스는 왼손(`d405_left`).

## 1. 자세 (단일 관절 이동)

| Task | 움직이는 부위 | 내용 |
|---|---|---|
| `ready_pose` | 토르소 + 양팔 | 기본 대기 자세 |
| `torso_up` | 토르소 | 토르소 상대 직선 이동 1회 |
| `object_gripping_initial_pose` | 양팔 | 물체 파지 준비 자세 (허리 그대로). 코드에서 `object_gripping_initial_pose('left'/'front'/'right')` 는 허리도 같이 맞춘다 |
| `object_gripping_initial_pose(front)` / `(left)` / `(right)` | 토르소 + 양팔 | 허리(yaw 0° / +90° / -90°)와 양팔 파지 준비 자세를 함께 맞춘다. 허리만 돌리는 `torso_align` 은 팔이 이니셜 자세가 아니면 충돌 위험이 있어 목록에서 뺐다 (`turn_and_move` 내부에서만 사용) |
| `search_pose` | 오른팔 | 문(tag_6) 탐색 자세 |
| `search_switch_pose` | 토르소 + 양팔 | 스위치(tag_5) 탐색 자세 |
| `door_handle_grasp_pose` | 토르소 + 양팔 | `torso_right` 자세에서 오른팔만 손잡이 파지 관절값 |

## 2. 데모

| Task | 팔 | 내용 |
|---|---|---|
| `impedance_demo_press` | 오른팔 | 관절 임피던스로 base -z 3 cm 접촉 후 5 cm 상승 |
| `impedance_demo_joint` | 오른팔 | 팔꿈치를 관절 임피던스로 10° 굽혔다가 위치제어로 복귀 |
| `wave_both_arms` | 양팔 + 토르소 | 그리퍼 닫고 양팔 들어 좌우로 3회 흔들기 (그리퍼 백그라운드 개폐). 빈손 전용 |

## 3. 문 / 스위치 (오른손)

| Task | 카메라 / 태그 | 내용 |
|---|---|---|
| `align_base_to_tag6` | d435 / tag_6 | tag_6 이 `d435_camera_center` 기준 측정 위치에 오도록 base 이동 (yaw 미보정) |
| `align_base_to_tag7` | d435 / tag_7 | `align_base_to_tag6` 과 같은 base 위치가 되도록 tag_7 기준 이동 |
| `move_ee_to_tag6_offset` | d435 / tag_6 | 오른손 TCP 를 tag_6 기준 손잡이 위치로 이동 |
| `door_open_right_arm_stream` | - | 오른손 원호 스트림 경로로 문 열기 (경첩 기준, 12 s) |
| `search_and_open_door2` | d435 / tag_6 | 그리퍼 열기 → `search_pose` → `align_base_to_tag6` → `move_ee_to_tag6_offset` → 그리퍼 닫기 → `door_open_right_arm_stream` |
| `search_and_push_switch` | d405_right / tag_5 | `search_switch_pose` → 그리퍼 닫기 → 태그 접근 → base 좌표 직선 누름/후퇴 → `search_switch_pose` |
| `open_door_and_push_switch2` | d435 + d405_right / tag_6, tag_5 | `search_and_open_door2` → 손 빼기 → `search_and_push_switch` |
| `push_switch_and_close_door` | d405_right + d435 / tag_5, tag_7 | 스위치 누르기 → 손 이동 → tag_7 기준 손잡이 이동 → 그리퍼 닫기 → 원호 스트림으로 문 닫기 |

## 4. 물체 이송 / 이동

| Task | 카메라 / 태그 | 내용 |
|---|---|---|
| `object_handover_demo_final` | d435 + d405_left / tag_0, tag_3 | 허리 왼쪽 자세 → `LH1` 에서 파지 → `LH2` 로 이동 → 배치 (왼손) |
| `object_handover_demo_final_tmp` | 위와 동일 | `object_handover_demo_final` 과 내용이 완전히 같음 |
| `move_zssw_to_zs_and_put_down` | - | `ZSSW` → `AP` → `ZS` base 이동 (허리 정면 전환 포함). **왼손 내려놓기 단계는 주석 처리됨** |
| `move_zs_to_zssw` | - | `ZS` → `AP` → `ZSSW` 역경로 이동 |
| `test1` | d405_left / tag_3, tag_0 | 왼손만. base·허리 고정(허리 왼쪽 자세). tag_3 잡기 → 든 높이로 왼팔만 가슴 기준 왼쪽 15 cm(base -x) → tag_0 놓기 → 다시 잡기 → 오른쪽 15 cm(+x) 복귀 → tag_3 놓기. 10 회 반복 |
| `push_switch_and_close_door_move_to_zs` | d435 + d405_right / tag_5, 6, 7 | `open_door_and_push_switch2` → 이동 전 자세 → `move_zssw_to_zs_and_put_down` → `move_zs_to_zssw` → `push_switch_and_close_door` |

## 등록되지 않은 helper (다른 Task 안에서 사용)

| 함수 | 내용 |
|---|---|
| `pick_up_object_at(loc, arm=...)` / `put_down_object_at(loc, arm=...)` | 지점의 태그로 base 정렬(d435) 후 손목 D405 로 접근·접촉, 파지/배치. **`arm` 필수** (기본 손 없음). 그리퍼·손목 카메라는 arm 에서 자동 선택 |
| `align_base_and_pick_up_object` / `align_base_and_put_down_object` | 위 두 함수의 태그 ID 버전 |
| `pick_up_move_and_put_down_object(a, b, arm=...)` | a 에서 파지 → 이동 → b 에서 배치 |
| `pick_up_move_and_put_down_object2(a, b, arm=...)` | 직전 도착 지점에서 a 로 먼저 이동한 뒤 위와 동일 |
| `pick_up_object_in_place(tag, arm=...)` / `put_down_object_in_place(tag, arm=...)` | base 이동 없이 현재 자세에서 손목 카메라로 파지/배치 + 7.5 cm 상승. `align_base_and_*` 도 내부에서 이것을 쓴다 |
| `turn_and_move(a, b)` | 필요하면 허리 방향 전환 후 base 이동 |
| `turn_left_and_move_left` / `turn_right_and_move_right` | 허리 ±90° 후 base ±0.87 m 이동 |
| `move_ee_to_tag7_offset`, `door_close_right_arm_stream` | tag_7 손잡이 이동, 문 닫기 원호 스트림 |
| `pose_before_base_movement` | 중간 자세 |

## 정리 필요 항목

- `object_handover_demo_final_tmp` 가 `object_handover_demo_final` 과 동일하다. 하나는 지워도 된다.
- `__all__` 에 쉼표가 빠져 `'torso_up' 'JS_SOFT_ARM'` 이 `'torso_upJS_SOFT_ARM'` 하나로 합쳐져 있다. `from rby1_planner.task import *` 를 하면 에러가 난다 (UI 로딩은 `build_tasks()` 를 쓰므로 영향 없음).
