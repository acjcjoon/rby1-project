# UPC / LAB 양쪽 시간 계측

주행·VSLAM timeout을 변경하지 않고 데이터를 수집한다. UPC와 LAB에서 같은 trial 이름으로 실행하고, JSONL/rosbag/pcap과 자동 요약 보고서로 구간별 지연과 정지 원인을 비교한다. `/rby1/vslam/timing`에는 UPC stereo enqueue, LAB receive/publish, cuVSLAM pose callback, UPC pose receive/publish, PoseAdapter receive/publish 경계가 원본 stamp와 TCP session ID로 기록된다. bridge 상태에는 송·수신 latest-only mailbox의 enqueue/drain/replace/expire 누적 카운터도 포함된다.

## 실행

먼저 평소대로 stack을 실행한다. 각 PC의 별도 터미널에서 해당 ROS와 workspace setup을 source한다. **수집 터미널의 ROS_DOMAIN_ID는 해당 PC에서 실행 중인 stack과 같아야 한다.** UPC와 LAB끼리 같은 도메인일 필요는 없으며, 두 PC 사이 VSLAM 전송은 TCP다.

UPC (Jetson/Humble):

```bash
source /opt/ros/humble/setup.bash
source ~/rby1_ros2_ws/install/setup.bash
cd ~/rby1_ros2_ws/src/rby1-project/rby1_vslam
bash scripts/capture_vslam_upc.sh --run-id point1_point2_01
```

LAB (Isaac ROS Jazzy 컨테이너 내부):

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
cd /workspaces/isaac_ros-dev/src/rby1_vslam
bash scripts/capture_vslam_lab.sh --run-id point1_point2_01
```

스크립트는 ROS_DOMAIN_ID를 덮어쓰지 않는다. 예를 들어 LAB stack이 85로 실행 중이면 이 수집 터미널에도 `export ROS_DOMAIN_ID=85`를 적용한다. UPC 값은 실제 stack을 따른다.

공통 수집 파일 `capture_vslam_timing.sh`, 메타데이터 관측기 `timing_observer.py`, 분석기 `analyze_vslam_timing.py`, 플로터 `plot_vslam_timing.py`, role wrapper를 함께 복사한다. 내부 경계 이벤트를 얻으려면 수정된 `rby1_vslam`을 **UPC와 LAB 양쪽에서 다시 빌드/반영**해야 한다. 수집·분석은 추가 pip 패키지 없이 실행되고, PNG 생성에만 matplotlib가 필요하다.

두 PC 모두 수집이 시작됐는지 확인한 뒤 정지 10초→Point 1→Point 2→정지/취소 후 10초를 포함해 기록한다. status=4를 확인한 뒤 다음 goal을 보낸다. 각 수집 터미널에서 Ctrl+C로 종료하면 수집 자식 프로세스만 정리하고 해당 PC의 단독 `timing_report.txt/json`을 자동 생성한다. 수집 프로세스는 이동/enable/cancel/전원 서비스를 호출하지 않는다.

로그의 metadata/rosbag 구독 정보를 확인한다. `observer_health.counts`에 영상·포즈 토픽 수가 늘어나는지 확인하면 도메인 불일치 또는 없는 토픽을 발견할 수 있다. 현재 topic 이름은 저장소의 기본 launch 구성에 맞춰져 있다. 별도 remap을 사용한다면 관측기의 목록도 그 구성에 맞춰야 한다.

## 저장 파일

기본 디렉터리: `~/rby1_timing/<run-id>_<upc|lab>_<시간>_<고유값>/`. `--output /경로`로 부모 폴더를 지정할 수 있다.

| 파일 | 내용 |
|---|---|
| `events.jsonl` | 메시지별 원본 stamp·관측 시각·영상 크기·포즈/속도·상태 JSON |
| `timing_report.txt` | 종료 시 자동 생성되는 사람이 읽는 단독 PC rate/gap/drop/상태 진단 |
| `timing_report.json` | 위 결과의 기계 판독용 원본 통계와 exact-stamp 구간 매칭 결과 |
| `timing_samples.csv` | session/stamp별 전체 지연과 누적 막대용 각 구간 ms |
| `analysis.log` | 자동 분석기의 stdout/stderr. 보고서 생성 실패 원인도 여기에 남음 |
| `bag/` | rosout, TF, 진단, bridge, 포즈. UPC는 gate/localization/control/속도/action 상태·feedback도 포함 |
| `environment.txt` | role, 공통 run_id, 로컬 capture_id, domain, ROS 배포판, 호스트, 최초 시계 상태 |
| `*_params.yaml` | 실제 노드 파라미터와 timeout. 없는 노드나 조회 실패는 그 파일에 기록 |
| `topics.txt`, `nodes.txt`, `cmd_raw_publishers.txt` | 실행 그래프와 속도 발행자 목록 |
| `system_samples.txt` | wall/monotonic 시각, NTP 상태, load, NIC 누적 바이트/오류, TCP ss, 프로세스 CPU/메모리, 가능한 GPU 이용률 |
| `metadata.log`, `rosbag.log` | 수집기 구독·경고·실패 정보 |
| `tcp.pcap`, `tcpdump.log` | `--pcap`일 때만 생성되는 TCP 헤더 위주의 캡처와 캡처 drop 통계 |

## UPC와 LAB을 합친 자동 병목 보고서

각 PC에서 캡처를 종료한 뒤 두 결과 디렉터리를 한 PC에 모아 다음을 실행한다. 인자 순서는 상관없고, 정확히 UPC 하나와 LAB 하나여야 한다.

```bash
python3 scripts/analyze_vslam_timing.py \
  ~/rby1_timing/point1_point2_01_upc_YYYYMMDD_HHMMSS_XXXXXX \
  ~/rby1_timing/point1_point2_01_lab_YYYYMMDD_HHMMSS_XXXXXX \
  --output-dir ~/rby1_timing/point1_point2_01_combined
```

`timing_report.txt`의 첫 `Likely bottlenecks / events` 절에 500 ms 초과 gap, cuVSLAM non-tracking, TCP 재연결, stereo sync drop, TX mailbox 영상/pose 교체, Nav2 status 5를 우선 표시한다. 이어지는 표는 입력 영상→LAB 영상→LAB pose→UPC 반환 pose→PoseAdapter 결과 순서의 rate와 최대 gap, exact source stamp 매칭 지연을 보여준다. 임계값을 바꿔 비교하려면 `--stale-ms 750`처럼 지정한다.

세션별 그래프는 합친 보고서에서 오프라인으로 만든다.

```bash
python3 scripts/plot_vslam_timing.py --report /path/to/timing_report.json \
  --output-dir /path/to/timing_plots --threshold-ms 500
```

Windows에서도 ROS 없이 `py -m pip install matplotlib` 후 같은 명령을 실행할 수 있다.
`total_latency_by_session.png`은 프레임별 총 지연과 500 ms 빨간 점선,
`session_stage_breakdown.png`은 세션별 평균 총 지연을 다음 구간으로 나눈 누적 막대다.

1. UPC TX queue/socket
2. TCP 왕복 residual
3. LAB receive/publish
4. cuVSLAM
5. LAB TX queue/socket
6. UPC receive/DDS
7. PoseAdapter

총 지연은 UPC의 monotonic clock 하나로 측정한다. 개별 명시 구간도 각 호스트의 monotonic
duration이며, 이를 총 지연에서 뺀 TCP 왕복 residual도 PC 간 시계 offset에 영향을 받지 않는다.
반면 `one_way_wall_clock_diagnostic.png`의 UPC→LAB/LAB→UPC 편도 값은 wall clock이므로 NTP
offset/dispersion을 확인한 뒤에만 해석한다.

두 PC 사이 구간은 wall clock으로 계산되므로 `environment.txt`의 NTP 상태를 먼저 확인한다. 시계 offset이 불명확해도 각 PC 내부 monotonic gap, 매칭률, mailbox 교체 및 topic rate 비교는 사용할 수 있다.

JSONL은 오프라인 `json.loads()`로 읽어 CSV에 옮길 수 있는 원시 행이다. JSONL 안에는 start/end/observer_health/subscription_unavailable 이벤트도 있으므로 메시지 분석에서는 `event == "message"`로 필터링한다. 영상 픽셀·IMU 전체 배열은 JSONL에 저장하지 않는다. rosbag에도 영상 원본은 넣지 않는다.

### JSONL 필드

| 필드 | 단위·의미 |
|---|---|
| `role`, `host`, `capture_id`, `topic`, `type` | 관측 위치/실행/메시지 구분 |
| `observer_index` | 해당 관측기의 해당 토픽 수신 카운터. 송신 seq나 TCP seq가 아님 |
| `source_stamp_ns` | 원본 ROS header.stamp, 정수 ns. header가 없는 메시지에는 없음 |
| `observed_wall_ns` | 이 관측기 콜백 시작의 time.time_ns(), Unix 시각 |
| `observed_monotonic_ns` | 이 PC 관측기 콜백 시작의 monotonic ns |
| `observed_ros_ns` | 관측 노드의 ROS clock. 관측기는 기본 system time 사용 |
| `frame_id`, `child_frame_id` | 측정 프레임과 포즈 대상 |
| `width`, `height`, `step`, `encoding`, `payload_bytes` | 영상 메타데이터 및 원본 픽셀 바이트 수 |
| `position`, `orientation`, `velocity` | pose/twist가 있을 때 작은 수치 배열 |
| `state` | bridge/localization/gate 등의 JSON 전체. session_id, detail, drop 및 `tx_mailbox`/`rx_mailbox` enqueue·drain·replace·expire 카운터 보존 |
| `bridge_session_id`, `bridge_connected` | 관측 당시 bridge 상태. timing 이벤트는 wire packet의 정확한 session ID 우선 |
| `timing_stage` | `/rby1/vslam/timing` 내부 경계 이름 |

ns 정수는 CSV 생성 시 **int64 또는 문자열로 유지**한다. Unix ns를 float로 바꾼 뒤 빼면 정밀도를 잃는다. 먼저 정수끼리 뺀 차이에 `/1e6`을 적용해 ms로 변환한다. source stamp가 0이거나 없는 값은 유효한 source latency 표본으로 취급하지 않는다.

`observed_*`는 별도의 구독자 콜백 관측 시각이라 DDS 배달·executor 대기도 포함한다. 반면 timing 토픽의 `state.wall_ns`와 `state.monotonic_ns`는 bridge/PoseAdapter가 해당 경계에서 찍은 내부 시각이다. Humble/Jazzy 공통 호환을 위해 DDS MessageInfo에는 의존하지 않는다. [Humble executor 구현](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/executors.py)

## 어떤 구간을 계산할까

같은 `topic`과 `source_stamp_ns`의 UPC/LAB 메시지를 대응시킨다. 좌/우 영상은 topic별로 분리하고, 한쪽 영상과 CameraInfo stamp를 섞지 않는다. 최초 양쪽 bridge의 session_id가 같은지 확인하고, 재연결/시간 역행 경계에서 구간을 분리한다. source stamp가 반복되면 관측 인덱스·세션 문맥으로 중복을 확인한다. 시간 재설정 뒤 같은 stamp가 생겼는데 무조건 병합하지 않는다.

| 지표 | 계산 | 해석 |
|---|---|---|
| 영상 원본 표본 주기 | 같은 topic의 Δsource_stamp_ns | 카메라가 생성한 표본 간격 |
| 로컬 관측 간격 | 같은 topic의 Δobserved_monotonic_ns | 도착 빈도·지터·버스트. PC 간 monotonic 차이는 계산하지 않음 |
| UPC 영상 age | UPC observed_wall − 영상 source_stamp | 원본 stamp가 UPC system time과 정합될 때 센서/드라이버/DDS 관측 지연 |
| LAB 영상 age | LAB observed_wall − 영상 source_stamp | 시계 정합 시 카메라부터 LAB 관측까지 누적 지연 |
| UPC→LAB 영상 구간 | 동일 stamp의 LAB observed_wall − UPC observed_wall | 브리지/TCP/재발행/DDS 관측 차이. 순수 네트워크 지연은 아님 |
| LAB 입력→포즈 출력 | 동일 기준 stamp의 LAB pose observed_monotonic − LAB left-image observed_monotonic | cuVSLAM 파이프라인·스케줄링 관측 구간. 알고리즘 함수 실행시간은 아님 |
| LAB→UPC 포즈 구간 | 동일 pose topic/stamp의 UPC observed_wall − LAB observed_wall | 반환 브리지/TCP/DDS 관측 차이 |
| UPC 카메라→base 변환 | 동일 stamp의 UPC slam_odom observed_monotonic − camera_slam_odometry observed_monotonic | PoseAdapter·TF 대기를 포함한 관측 구간 |
| UPC 포즈 총 age | UPC slam_odom observed_wall − source_stamp | 0.5초 검사 기준과 비교할 end-to-end 측정 후보 |

cuVSLAM pose stamp는 프로젝트에서 덮어쓰지 않지만 모든 입력에 출력이 하나씩 존재한다고 가정하지 않는다. `override_publishing_stamp`나 Isaac 버전이 다르면 exact stamp match가 가능한지 먼저 검증한다. 좌우 스테레오 stamp도 최대 slop 차이가 있으므로 pose 출력이 어떤 입력 stamp를 쓰는지 실제 자료로 확인한다. exact match가 없을 때 무조건 nearest를 붙여 가짜 latency를 만들지 않는다.

같은 PC에서도 서로 다른 구독 콜백의 scheduling 순서로 음수 구간이 생길 수 있다. 내부 작업 시작/끝의 정확한 latency로 해석하지 않고 pcap·RMW·호스트 부하를 함께 비교한다.

관측 누락은 패킷 손실과 같지 않다. TCP 재전송, 브리지 latest-only 큐 폐기, 동기화 실패, cuVSLAM 출력률, 구독 QoS drop, 수집기 지연이 모두 가능하다. `bridge_status`의 dropped_stereo, imu_overflows, sync_drops 및 observer counts를 함께 본다. drop 카운터는 재연결 시 의미/연속성이 달라질 수 있으므로 session별로 비교한다.

## TCP 패킷도 확인할 때

선택적으로 두 PC에 `tcpdump`가 있고 해당 사용자가 캡처 권한을 가진 경우 다음 옵션을 붙인다. 스크립트는 sudo를 자동으로 실행하지 않는다. 권한이 없으면 tcpdump 로그에 실패가 남고 수집 전체를 종료하므로 먼저 캡처 권한을 준비하거나 `--pcap`을 빼고 실행한다.

```bash
bash scripts/capture_vslam_upc.sh --run-id point1_point2_01 --pcap --interface eth0
bash scripts/capture_vslam_lab.sh --run-id point1_point2_01 --pcap --interface eth0
```

eth0는 예시이며 실제 유선 NIC 이름을 넣는다. 포트가 다르면 `--port 7447` 값을 실제 값으로 바꾼다. LAB 컨테이너가 호스트 네트워크를 공유하지 않으면 컨테이너의 가상 NIC 관측 지점과 호스트 유선 NIC는 다르다. 호스트에서도 패킷을 캡처해 이 경계를 분리해야 할 수 있다. 기본 any 인터페이스는 편하지만 veth/bridge의 중복 관측이 발생할 수 있다.

pcap은 snaplen 128로 TCP 헤더와 작은 일부 payload만 저장한다. 전체 영상이나 애플리케이션 패킷 내용을 복원할 용도는 아니다. 원래 wire protocol 한 패킷이 여러 TCP segment로 나뉘고 여러 애플리케이션 패킷이 한 segment에 합쳐질 수 있으므로 “TCP segment 하나 = 영상 한 장”으로 해석하지 않는다. GSO/GRO/TSO offload도 캡처에서 보이는 segment 모양을 바꿀 수 있다.

오프라인 Wireshark/tshark에서 TCP RTT, retransmission, zero-window, ACK 지연, throughput을 볼 수 있다. 예시 CSV:

```bash
tshark -r tcp.pcap -Y tcp -T fields -E header=y -E separator=, \
  -e frame.time_epoch -e ip.src -e ip.dst -e tcp.srcport -e tcp.dstport \
  -e tcp.seq_raw -e tcp.ack_raw -e tcp.len -e tcp.window_size_value \
  -e tcp.analysis.ack_rtt -e tcp.analysis.retransmission \
  -e tcp.analysis.zero_window > tcp.csv
```

IPv6이면 ip.src/ip.dst 대신 ipv6.src/ipv6.dst도 포함한다. seq/ack는 양쪽 pcap 간 비교를 위해 raw 값을 쓴다. 연결의 4-tuple과 SYN/재연결 구간으로 분리한다. 재전송·offload·중복 캡처 때문에 단순 seq join만으로 모든 segment가 일대일 대응하지 않을 수 있다.

**시계 차이를 모르면 정확한 PC 간 편도 지연은 계산할 수 없다.**

`LAB 관측시각 − UPC 관측시각 = 실제 관측 구간 + LAB/UPC 시계 오프셋`이다. 같은 PC의 RTT·구간은 상대적으로 보기 쉽지만, wall clock이 조정되면 PC 간 그래프에 가짜 점프가 생긴다. monotonic은 각 PC 내부 간격에만 사용한다. chronyc의 offset/dispersion 또는 timesyncd 상태를 양쪽에서 저장하고 신뢰도 범위를 표시한다.

컨테이너에 chronyc/timedatectl이 없거나 NTP daemon 접근이 안 되면 해당 시계 상태는 저장되지 않는다. LAB 호스트에서 별도로 NTP 상태를 남겨야 한다. 시계를 바꾸거나 동기화 설정을 수정하는 동작은 수집 스크립트에 없다.

## 생성되는 그래프와 추가 권장 그래프

1. 기본 플로터가 세션별 UPC enqueue→base pose 총 지연과 **500 ms 선**을 표시한다.
2. 기본 플로터가 세션별 평균 구간 누적 막대와 total p95 마커를 표시한다.
3. 카메라/포즈 source 주기와 로컬 관측 간격을 그려 멈춤 또는 몰아서 도착하는 구간을 찾는다.
4. tracking_ok, localization healthy, gate enabled/fault, cancel_pending, action status를 같은 시간축에 그린다.
5. TCP RTT/retransmission/send-q/recv-q 및 CPU/GPU 부하를 겹쳐 지연 상승의 위치를 좁힌다.
6. nav2_cmd_vel→cmd_raw→cmd_vel을 함께 그려 어떤 계층에서 0으로 바뀌었는지 확인한다. Twist에는 원본 stamp가 없으므로 exact message latency 대응이 아니라 상태/속도 시계열 비교로 사용한다.

### 관측 패턴으로 원인 좁히기

| 패턴 | 우선 후보 |
|---|---|
| UPC 영상 자체의 source stamp/빈도 이상 | 카메라/드라이버/시계 |
| UPC 영상 정상, LAB 영상 age 증가 + TCP 문제 | 송신 큐·네트워크·수신 측 처리 |
| LAB 영상 정상, LAB 포즈 출력 지연/끊김 | cuVSLAM 추적·연산 부하 |
| LAB 포즈 정상, UPC camera pose 지연 | 반환 TCP·브리지·UPC scheduling |
| UPC camera pose 정상, base pose 누락 | 측정 시각 camera→base TF / PoseAdapter |
| base pose 정상, localization unhealthy | wheel TF, 지도 보정 jump, 세션 |
| 상태 정상인데 최종 cmd_vel=0 | rby1_control 안전 상태 또는 명령 watchdog |

## 관측의 한계와 검증

구독자는 영상 픽셀을 디스크에 저장하지 않지만 영상 메시지 자체는 DDS로 받아 역직렬화하므로 CPU/메모리 부하를 추가한다. rosbag도 작은 토픽 구독과 디스크 부하를 더한다. 수집기를 켠 경우/끈 경우의 동작을 비교하고 observer_health의 1초 heartbeat 간격과 topic counts로 수집기 자체 지연을 점검한다. 수집기의 센서 QoS는 best-effort이므로 관측 누락을 pipeline 손실로 단정하지 않는다.

system_samples의 ps pcpu는 프로세스 수명 평균이며 순간 CPU 정밀 지표가 아니다. GPU는 nvidia-smi가 있는 LAB에서 기록되며 Jetson에는 값이 없을 수 있다. ss는 낮은 빈도 snapshot이고 system sampling 주기도 명령 수행 시간+sleep 1초이므로 정확히 1 Hz라고 가정하지 않는다.

기존 bridge/transport/cuVSLAM/Nav2/control 실행 로직은 바꾸지 않았다. 로컬에서 Bash 구문, 도움말, ROS 없는 메타데이터 단위 검사를 수행한다. 실제 Humble/Jazzy 양쪽 통합 수집과 데이터 품질은 실험 시 구독/카운터/로그로 확인해야 한다.
