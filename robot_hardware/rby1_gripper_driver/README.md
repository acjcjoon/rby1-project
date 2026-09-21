# RBY1 Gripper Driver

RBY1 기본 그리퍼를 ROS 2 인터페이스로 노출하는 드라이버 패키지입니다.
`transport:=real`은 `rby1_sdk.DynamixelBus`, `transport:=sim`은 Isaac Sim의
그리퍼 UDP bridge를 사용합니다. open/close 정책은 `rby1_control` 백엔드가
담당하고, 이 패키지는 transport·calibration·상태와 정규화된 0~1 명령만
다룹니다. 두 transport는 동일한 코어와 ROS topic/service를 사용합니다.

양쪽 그리퍼가 모두 활성화되어 있으며 명령과 상태 배열은 항상
`[right, left]` 순서를 사용합니다.

```text
control frontend / planner
        |
rby1_control backend
        |
/rby1/gripper/command
        |
rby1_gripper_driver
        |
        +-- transport:=real -> Dynamixel: ID 0=right, ID 1=left
        |
        +-- transport:=sim  -> Isaac UDP: ID 1=right, ID 0=left
```

Real mode의 노드는 `/dev/rby1_gripper`가 보이는 U-PC에서 실행해야 합니다.
그리퍼 command/state는 `rby1_driver`나 robot RPC를 거치지 않고 이
serial device로 직접 오갑니다. 동일한 serial device를 다른 프로그램과
동시에 열면 안 됩니다.

단, **통신 경로와 전원 경로는 별개**입니다. 그리퍼 모터에는 양쪽
tool-flange의 12 V 출력이 필요합니다. 전원이 꺼져 있다면 기존 로봇 기동
절차나 `rby1_driver`의 robot/tool-flange power service로 먼저 켜야
합니다. Control UI 자체는 필수 조건이 아닙니다.

## ROS interface

기본 launch namespace는 `/rby1`입니다. 모든 2원소 배열의 순서는
`[right, left]`입니다. 실물은 `ID 0=right, ID 1=left`, Isaac UDP는
`ID 0=left, ID 1=right`이므로 driver가 transport별로 ID를 매핑합니다.

| Kind | Name | Type | Contract |
|---|---|---|---|
| Subscribe | `/rby1/gripper/command` | `std_msgs/msg/Float64MultiArray` | `[right, left]`, `0.0=open`, `1.0=closed` |
| Publish | `/rby1/gripper/state` | `std_msgs/msg/Float64MultiArray` | `[right, left]` normalized measurement |
| Publish | `/rby1/gripper/motor_state` | `sensor_msgs/msg/JointState` | 양쪽의 실제 위치(rad), 속도(rad/s) |
| Publish | `/rby1/gripper/ready` | `std_msgs/msg/Bool` | command를 받을 준비가 되었는지 표시 |
| Publish | `/rby1/gripper/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 장치, SDK, 통신, calibration, 전류, 온도 |
| Service | `/rby1/gripper/home` | `std_srvs/srv/Trigger` | 양쪽 endpoint calibration |
| Service | `/rby1/gripper/torque_enable` | `std_srvs/srv/SetBool` | 양쪽 motor torque on/off |

command는 길이가 정확히 2이고 각 값이 유한한 0~1 범위일 때만 받습니다.
`ready=true`일 때만 command가 실행됩니다.
Calibration 전에는 정규화된 close ratio가 정의되지 않으므로 `gripper/state`
대신 `gripper/motor_state`의 raw encoder feedback만 publish합니다.

`ready=true` 조건은 다음을 모두 만족하는 것입니다.

- 선택한 transport 초기화 성공
- 양쪽 motor ping 성공(실물 ID 0/1, sim ID 1/0)
- 유효한 calibration 존재
- 양쪽 torque enable
- homing 중이 아님
- 마지막 통신 상태가 정상(sim은 1초 이내 Isaac feedback 수신)

## Build

ROS 환경과 이 workspace가 사용하는 SDK 환경을 적용한 뒤 빌드합니다.

```bash
export RBY1_SDK_PATH=/root/sdk/rby1-sdk
source /opt/ros/humble/setup.bash
cd ~/rby1_ros2_ws

colcon build --symlink-install --packages-select rby1_gripper_driver
source install/setup.bash
```

전체 workspace 빌드가 필요하면 다음을 사용합니다.

```bash
colcon build --symlink-install --cmake-clean-cache
source install/setup.bash
```

## Isaac Sim에서 양쪽 그리퍼 검증

Isaac Sim과 driver는 별도 terminal에서 실행합니다. 아래는 실행 가이드이며,
이 저장소 수정 과정에서는 Isaac Sim을 직접 실행하지 않았습니다.

Isaac terminal에서 body command를 보내지 않고 gripper bridge만 받도록
`--udp`와 `--gripper`를 함께 사용합니다. 이 저장소에서 확인한 현재 runner는
`--gripper` 사용 시 `--sim-gripper`도 자동으로 켭니다.

```bash
cd ~/rby1-sim-isaac
./docker/run.sh --udp --gripper --gripper-name rb_gripper
```

Driver terminal에서는 같은 ROS 패키지를 sim transport로 시작합니다. 첫
검증은 fake homing까지 동일한 코어 경로로 통과하도록 `auto_home:=true`를
사용할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
source ~/rby1_ros2_ws/install/setup.bash

ros2 launch rby1_gripper_driver gripper_driver.launch.py \
  transport:=sim auto_home:=true
```

기본 UDP 설정은 command `127.0.0.1:5007`, state `0.0.0.0:5008`입니다.
Isaac이 다른 host에 있으면 운영 YAML의 `sim_host`를 driver에서 Isaac host로,
Isaac 실행 인자의 `--gripper-state-ip`를 driver host로 지정해야 합니다.
같은 host의 기본 `docker/run.sh`는 `--network=host`를 사용하므로 기본값을
그대로 쓸 수 있습니다.

연결 상태는 다음으로 확인합니다.

```bash
ros2 topic echo --once /rby1/gripper/diagnostics
ros2 topic echo /rby1/gripper/state
```

정상이라면 diagnostics에 아래 값이 나타납니다.

```text
transport: sim
driver: IsaacSimGripperUDP
sim_feedback_received: true
state_source: isaac_feedback
active_motor_ids: [1, 0]
```

그다음 기존 debug controller 또는 Control UI에서 작은 변위의 `right`, `left`,
`both` 명령을 보냅니다. ROS 계약은 real mode와 완전히 같습니다.

```bash
ros2 run rby1_gripper_driver gripper_debug_controller \
  --ros-args -r __ns:=/rby1
```

플래너 operator UI까지 같이 검증하려면 별도 terminal에서 planner UI를
실행합니다.

```bash
ros2 launch rby1_planner planner_ui.launch.py
```

Isaac feedback이 `sim_feedback_timeout_sec`(기본 1초) 동안 오지 않으면 driver는
상태를 재발행하지 않고 diagnostics를 ERROR로 바꿉니다. 포트 충돌이 있으면
초기화가 실패합니다. `transport:=real`과 `auto_home:=true`가 기본이므로 인자
없이 launch하면 실물 양쪽 자동 homing이 시작됩니다.

## 실물 검증 전 준비

기본 launch는 실물에서도 시작 즉시 양쪽 자동 homing을 수행합니다.
Driver를 시작하기 전에 양쪽의 물체와 payload를 제거하고 전체 이동 범위를 비우며,
비상정지를 즉시 누를 수 있어야 합니다. 자동 이동이 허용되지 않는 상황에서는
반드시 `auto_home:=false`로 실행합니다.

Tool-flange 12 V가 꺼진 상태에서 driver를 먼저 시작하면 프로세스는 종료되지 않고
`NOT READY`로 대기합니다. UI에서 **12V ON**을 누른 뒤 양쪽 전압 표시가 12 V로
확인되어 **HOME**이 활성화되면, 양쪽 이동 범위를 비우고 HOME을 누릅니다. 이때
serial 초기화, 양쪽 homing, 현재 위치 hold가 순서대로 완료되면 `ready=true`가
됩니다. 그리퍼 전원에는 반드시 12 V만 사용합니다.

그리퍼 serial port는 driver 한 프로세스만 사용해야 합니다. SDK 예제,
teleoperation 프로그램 또는 이전 driver 프로세스를 동시에 실행하지
마십시오.

### 1. 로봇과 그리퍼 전원 확인

연구실의 기존 실물 기동 절차로 로봇 전원과 양쪽 tool-flange 12 V가 이미
켜져 있다면 이 단계에서는 상태만 확인합니다. `rby1_driver`를 이용해
전원을 켜는 경우 먼저 실제 로봇 주소로 driver를 실행한 뒤 service 이름을
찾습니다.

```bash
ros2 service list | grep tool_flange_power
ros2 service list | grep robot_power
```

기본 namespace가 `/rby1`인 경우의 예시는 다음과 같습니다. 실제
`ros2 service list` 결과가 `/tool_flange_power`처럼 다르면 그 이름을
사용합니다.

```bash
# 48 V가 이미 켜져 있다면 첫 호출은 생략합니다.
ros2 service call /rby1/robot_power rby1_msgs/srv/StateOnOff \
  "{state: true, parameters: '48v', value: 0.0}"

ros2 service call /rby1/tool_flange_power rby1_msgs/srv/StateOnOff \
  "{state: true, parameters: '12v', value: 0.0}"
```

tool-flange state publish가 활성화되어 있으면 양쪽 출력 전압도 확인합니다.

```bash
ros2 topic echo --once /rby1/tool_flange/right
ros2 topic echo --once /rby1/tool_flange/left
```

`rby1_driver`는 여기서 전원 설정/모니터링에만 사용됩니다. 이후 그리퍼
명령은 gripper driver가 U-PC의 Dynamixel serial로 직접 보냅니다. 따라서
robot driver와 gripper driver를 함께 실행해도 되지만, SDK gripper 예제처럼
`/dev/rby1_gripper`를 여는 다른 프로세스는 함께 실행하면 안 됩니다.

### 2. 장치와 Python SDK 확인

U-PC에서 실행합니다.

```bash
ls -l /dev/rby1_gripper
readlink -f /dev/rby1_gripper
python3 -c "import rby1_sdk; print(rby1_sdk.__file__); print(rby1_sdk.upc.GripperDeviceName); print(rby1_sdk.DynamixelBus)"
```

기대 결과:

- `/dev/rby1_gripper`가 실제 tty 장치를 가리킴
- Python import 성공
- 기본 device name이 `/dev/rby1_gripper`
- `DynamixelBus` type이 출력됨

장치를 다른 프로세스가 사용 중인지 확인할 수 있습니다.

```bash
fuser -v /dev/rby1_gripper
```

gripper driver를 실행하기 전에는 불필요한 PID가 없어야 합니다. `fuser`가
설치되지 않았다면 이 검사는 생략하고, 관련 예제와 node를 모두 종료합니다.

Docker 안에서 gripper driver를 실행한다면 컨테이너 시작 시 실제 장치를 넘겨야
합니다.

```text
--device=/dev/rby1_gripper:/dev/rby1_gripper
```

컨테이너 안에서도 `ls -l /dev/rby1_gripper`와 SDK import가 성공해야
합니다. 일반 사용자로 실행할 때는 해당 장치의 group 권한도 필요합니다.

첫 검증에서는 gripper driver와 debug controller를 같은 컨테이너에서 실행하는 것이
가장 단순합니다. 서로 다른 컨테이너에서 실행한다면 두 컨테이너의
`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, DDS 설정과 network mode가
일치해야 합니다.

```bash
printenv ROS_DOMAIN_ID
printenv RMW_IMPLEMENTATION
ros2 node info /rby1/gripper_driver
ros2 topic info -v /rby1/gripper/command
```

노드는 보이는데 CLI graph가 서로 다르면 각 환경에서
`ros2 daemon stop` 후 `ros2 daemon start`를 실행해 cache를 새로 만든
뒤 다시 확인합니다. 그래도 다르면 gripper driver와 debug controller를 같은
컨테이너에서 먼저 검증해 DDS/container 문제와 하드웨어 문제를 분리합니다.

### 3. Driver 시작과 자동 homing

아래 명령을 실행하면 port open과 양쪽 ID 0/1 ping 직후 자동 homing이 시작됩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/rby1_ros2_ws/install/setup.bash

ros2 launch rby1_gripper_driver gripper_driver.launch.py \
  transport:=real auto_home:=true
```

실물 양쪽이 음의 토크 endpoint를 먼저 찾고 양의 토크 endpoint를 찾습니다.
`closed_at_minimum`의 `[right, left]` 설정에 따라 각 endpoint를 정규화합니다.
성공하면 현재 위치에서 holding을 시작하고 노드가 다음 상태로 준비됩니다.

```text
transport=real
endpoint=/dev/rby1_gripper
baud=2000000
active IDs=(0, 1)
ready=true
```

노드와 실제 측정 상태를 확인합니다.

```bash
ros2 node list
ros2 topic echo --once /rby1/gripper/ready
ros2 topic echo --once /rby1/gripper/diagnostics
ros2 topic echo --once /rby1/gripper/motor_state
```

정상적인 초기 diagnostics의 핵심 값은 다음과 같습니다.

```text
initialized: true
healthy: true
torque_enabled: true
ready: true
message: Ready
active_motor_ids: [0, 1]
state_source: dynamixel_measurement
```

여기서 gripper driver가 종료된다면 launch terminal의 메시지로 원인을 구분합니다.

| 오류 | 우선 확인할 항목 |
|---|---|
| `rby1_sdk is not installed` | launch에 사용된 Python 환경과 SDK 설치 |
| `failed to open the gripper port` | device 경로, 권한, Docker device mount, 중복 사용 |
| `failed to set ... baud rate` | U2D2/USB 상태와 baud 설정 |
| `IDs did not respond` | 양쪽 tool-flange 12 V, RS485 배선, ID 0/1, baud 2000000 |

### 4. Debug controller에서 바로 제어

자동 homing이 성공한 뒤 새 terminal에서 debug controller를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/rby1_ros2_ws/install/setup.bash

ros2 run rby1_gripper_driver gripper_debug_controller \
  --ros-args -r __ns:=/rby1
```

`ready=true`가 보이면 바로 작은 변위부터 명령할 수 있습니다.

```text
state
right 0.90
left 0.90
open both
close both
```

다시 homing해야 하면 전체 이동 범위를 비운 뒤 같은 창에서 `home`을 입력합니다.

```bash
ros2 topic echo --once /rby1/gripper/ready
ros2 topic echo --once /rby1/gripper/diagnostics
```

Diagnostics에는 측정된 calibration min/max가 표시됩니다. CLI로 다시 homing할
수도 있습니다.

```bash
ros2 service call /rby1/gripper/home std_srvs/srv/Trigger "{}"
```

## Debug controller로 단계적 동작 확인

새 terminal에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/rby1_ros2_ws/install/setup.bash

ros2 run rby1_gripper_driver gripper_debug_controller \
  --ros-args -r __ns:=/rby1
```

먼저 현재 상태를 확인합니다.

```text
state
```

`published close ratios`는 ROS command를 보냈다는 뜻일 뿐, 모터가 목표에
도달했다는 뜻은 아닙니다. 매 단계마다 `state`, `motor_state`,
`diagnostics`의 실제 측정값을 함께 봐야 합니다.

디버깅할 때는 다음 세 단계를 구분합니다.

| 관측 결과 | 확인된 범위 |
|---|---|
| debug controller에 `published close ratios` 출력 | debug node가 ROS publish 호출 |
| diagnostics의 `target_close_ratio_right_left` 변경 | driver가 command를 수락하고 양쪽 SDK position write 호출 |
| `state`와 `motor_state.position` 변경 | Dynamixel encoder 기준으로 실제 모터가 이동 |

두 번째까지만 되고 세 번째가 안 되면 ROS topic 문제가 아닙니다. 이 경우
`torque_enabled`, tool-flange 12 V, Dynamixel current/temperature, 기구
걸림을 확인합니다. SDK의 group sync write는 명령 전송 함수이므로
`published` 로그 자체는 물리적 도달 확인이 아닙니다.

기본 실물 설정은 양쪽 모두 큰 encoder endpoint를 closed로 해석합니다. 먼저
각 그리퍼를 따로 약 10% 움직여 방향을 확인합니다.

```text
right 0.90
state
right 1.00
state
left 0.90
left 1.00
state
```

각 `0.90` 명령에서 선택한 쪽만 closed 위치로부터 약간 open 방향으로
움직여야 합니다. 열림/닫힘 방향이 반대라면 해당
`closed_at_minimum[right|left]` 값을 변경한 뒤 calibration 의미를 다시 검증합니다.

좌우와 방향이 맞은 뒤에만 범위를 단계적으로 확대합니다.

```text
set 0.75 0.75
state
set 0.50 0.50
state
set 0.25 0.25
state
set 0.00 0.00
state
set 0.25 0.25
set 0.50 0.50
set 0.75 0.75
set 1.00 1.00
state
```

다른 terminal에서 raw 측정값을 계속 볼 수 있습니다.

```bash
ros2 topic echo /rby1/gripper/motor_state
```

```bash
ros2 topic echo /rby1/gripper/diagnostics
```

다음이 보이면 즉시 명령을 중단하고 torque를 끕니다.

- 명령과 반대 방향으로 이동
- 오른쪽 gripper가 움직이거나 명령을 받은 흔적이 있음
- 기구 간섭, 이상음, 진동 또는 한쪽 정지
- 상태값이 변하지 않는데 전류가 계속 증가
- 온도가 빠르게 상승하거나 통신이 unhealthy로 변경

```bash
ros2 service call /rby1/gripper/torque_enable \
  std_srvs/srv/SetBool "{data: false}"
```

정상적인 `Ctrl-C` 종료도 양쪽 torque를 해제합니다.

## Calibration 저장과 재사용

성공한 homing 응답의 endpoint를 별도 운영 YAML에 저장하면 이후 자동 homing을
생략할 수 있습니다. 아래 숫자는 형식 예시이며 실제 로봇에 그대로 사용하면 안 됩니다.

```yaml
/**:
  ros__parameters:
    use_saved_calibration: true
    # 실제 homing 결과를 [right, left] 순서로 입력합니다.
    calibration_min_rad: [-1.15, -1.20]
    calibration_max_rad: [1.16, 1.18]
    closed_at_minimum: [false, false]
    endpoint_margin_ratio: 0.0
    auto_enable_torque: true
```

```bash
ros2 launch rby1_gripper_driver gripper_driver.launch.py \
  config:=/absolute/path/to/hardware.yaml auto_home:=false
```

저장된 calibration을 사용할 때 driver는 현재 encoder 위치를 먼저 읽고 그
위치에서 holding을 시작한 뒤 `ready=true`가 됩니다. Gripper 기구나 motor
설정, ID, 조립 방향이 바뀌었다면 저장값을 재사용하지 말고 작업 공간을 비운 뒤
명시적 homing을 다시 수행합니다.

`endpoint_margin_ratio`를 0보다 크게 설정하면 운영 command가 hard stop에
직접 닿지 않게 할 수 있습니다. 값을 바꾸면 유효 stroke와 0/1 의미를 다시
검증해야 합니다.

## `rby1_control` backend 연결

`rby1_control` backend는 `/rby1/gripper/command`에 `[right, left]`
목표를 publish합니다. frontend와 planner는 backend의 `right`, `left`,
`both` API를 사용할 수 있습니다. Backend는
다음 조건으로 driver 연결과 명령 완료를 판단합니다.

- `/rby1/gripper/ready == true`
- `/rby1/gripper/state`가 목표 tolerance 안에 도달
- timeout이 지나지 않음

종료나 fault 처리 시에는 `/rby1/gripper/torque_enable`을 사용해 torque를
해제할 수 있습니다.
