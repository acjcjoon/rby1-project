# 회전 후 고정 stereo VSLAM 비교 실험

한 번 녹화한 동일한 stereo/IMU 시퀀스를 반복 재생해 수동 복귀 오차를 제거하고,
회전 종료 후 영상 변화와 IMU가 정지 pose jitter에 미치는 영향을 분리한다. 전체 실행
명령과 터미널별 순서는 저장소 루트의 `test.md`를 따른다.

## 비교 조건

| 조건 | 회전까지 Stereo | 마지막 정지 Stereo | IMU |
|---|---|---|---|
| `baseline` | 녹화 시퀀스 | 녹화 시퀀스 | 녹화 시퀀스 |
| `no_imu` | 녹화 시퀀스 | 녹화 시퀀스 | 사용 안 함 |
| `fixed_stereo` | 녹화 시퀀스 | 정지 직후 pair 반복 | 녹화 시퀀스 |
| `fixed_stereo_no_imu` | 녹화 시퀀스 | 정지 직후 pair 반복 | 사용 안 함 |

고정 조건도 제자리 두 바퀴가 끝날 때까지는 원본 영상을 그대로 전달한다. source bag의
`returned_stationary` 마커가 수신된 뒤 첫 coherent 좌/우 image와 CameraInfo tuple을 잡아
남은 정지 구간에 반복 발행한다. 같은 이미지를 좌우에 복사하지 않는다.

## 입력 캡처

UPC에서 D435i와 수동 mobile-base 조작기를 실행한 뒤:

```bash
cd "$HOME/rby1_ros2_ws/src/rby1-project"

bash rby1_vslam/scripts/capture_vslam_replay_input.sh \
  --run-id spin2_source \
  --domain 10
```

스크립트 순서는 다음과 같다.

1. base와 카메라를 고정하고 Enter
2. 초기 정지 10초
3. 같은 방향으로 제자리 두 바퀴 회전
4. 완전히 멈춘 뒤 Enter
5. 최종 정지 10초

4번 Enter가 `returned_stationary` 마커이므로 완전히 멈춘 다음 눌러야 한다.

## 조건별 실행 원칙

- 조건마다 LAB cuVSLAM을 빈 map 상태로 다시 시작한다.
- `baseline`, `fixed_stereo`: LAB `enable_imu=true`
- `no_imu`, `fixed_stereo_no_imu`: LAB `enable_imu=false`
- 네 조건 모두 같은 source bag과 playback rate `1.0`을 사용한다.
- replay 중 live D435 publisher는 모두 종료한다.

UPC replay 명령의 condition만 바꾼다.

```bash
bash rby1_vslam/scripts/run_vslam_replay_condition.sh \
  --bag "$SOURCE_BAG" \
  --condition baseline \
  --lab-host "$LAB_IP" \
  --domain 10
```

유효한 condition은 다음 네 개다.

```text
baseline
no_imu
fixed_stereo
fixed_stereo_no_imu
```

## 해석

- 1↔2: 전체 구간의 IMU 효과
- 1↔3: IMU 사용 시 마지막 정지 영상 변화의 효과
- 2↔4: IMU 미사용 시 마지막 정지 영상 변화의 효과
- 3↔4: 영상 고정 시 IMU가 만드는 pose 변화

회전 구간은 누적 yaw, tracking loss, loop correction을 비교하고 마지막 10초는 pose 중심
이동량, frame-to-frame translation/yaw jitter p95·max, 저주파 wander를 비교한다.
