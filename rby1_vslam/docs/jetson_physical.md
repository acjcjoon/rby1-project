# RBY1: 새 NVMe SSD에서 Isaac ROS 3.2 설치부터 VSLAM 실행까지

NVMe를 추가 장착한 RBY1 U-PC에서 순서대로 실행하는 명령 가이드다.
현재 로봇의 OS/JetPack을 유지하면서 SSD에 Docker 이미지, Isaac workspace, 지도와 bag을 둔다.
아래 Linux 명령은 **RBY1 U-PC의 터미널**에서 실행한다. Windows 파일 전송 단계만 별도로 표시한다.
가이드의 명령을 로봇에서 실행한 결과는 아직 확인하지 않았다.

| 항목 | 이 가이드의 기준 |
|---|---|
| 하드웨어 | RBY1 U-PC: Jetson AGX Orin 64GB + AVerMedia D315 |
| 호스트 환경 | Ubuntu 22.04, JetPack 6.1/6.2 |
| ROS / Isaac ROS | Humble / release-3.2 |
| SSD 마운트 | `/mnt/nova_ssd` — 예시 경로 |
| 호스트 workspace | `/mnt/nova_ssd/workspaces/isaac_ros-dev` |
| 컨테이너 workspace | `/workspaces/isaac_ros-dev` |
| 카메라 | D435i, 3.2 공식 RealSense layer |

하드웨어는 [Rainbow Robotics 사양](https://rainbowrobotics.github.io/rby1-dev/hardware/electrical_part.html#u-pc-and-r-pc),
소프트웨어 조합은 [NVIDIA 3.2 지원 환경](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html)을 기준으로 한다.
이미지와 기본 데이터용으로 SSD에 최소 30GB 여유를 확보하고, 녹화 데이터 용량은 별도로
확보한다. 아래 표준 설치는 호스트 OS를 SSD로 다시 플래시하는 절차를 포함하지 않는다.

## 1. 먼저 JetPack과 SSD 장치를 확인한다 — 호스트

```bash
tr -d '\0' < /proc/device-tree/model; echo
cat /etc/nv_tegra_release
dpkg-query -W nvidia-jetpack 2>/dev/null || true
lsb_release -ds
uname -m
lsblk -o NAME,PATH,SIZE,MODEL,TYPE,FSTYPE,UUID,MOUNTPOINTS
findmnt /
```

`uname -m`은 `aarch64`여야 한다. JetPack metapackage가 없을 수도 있으므로 L4T 출력도 본다.
L4T `R36.4`, revision `0`은 JetPack 6.1, `R36.4`, revision `3`은 JetPack 6.2에 해당한다.
[JetPack 6.1](https://developer.nvidia.com/embedded/jetpack-sdk-61),
[JetPack 6.2](https://developer.nvidia.com/embedded/jetpack-sdk-62)
`R35.x`/JetPack 5 또는 다른 버전이면 SSD 설정까지만 진행하고, 이 3.2 환경 설치 전에
호스트 소프트웨어 호환을 확인한다. SSD를 추가해도 JetPack/CUDA 버전은 바뀌지 않는다.

RBY1의 D315는 NVIDIA 개발키트와 캐리어보드가 다르다. JetPack 변경이 필요하면 실제 보드
리비전과 RBY1 설정에 맞는 BSP를 사용한다. 일반 개발키트 플래시 명령을 그대로 적용하지 않는다.
[AVerMedia BSP](https://professional.avermedia.com/support/download)

아래에서 `nvmeXn1`은 **방금 추가한 SSD의 실제 장치명으로 바꾼다**.
`findmnt /`와 `lsblk`에 나타난 현재 부팅 디스크를 선택하지 않는다.

## 2. SSD를 준비하고 `/mnt/nova_ssd`에 마운트한다 — 호스트

### 2-A. SSD가 이미 ext4로 포맷돼 있으면

기존 ext4 파티션을 사용한다. 아래 파티션 예시를 실제 장치명으로 바꾼다.

```bash
VSLAM_SSD_PARTITION=/dev/nvmeXn1p1
test -b "$VSLAM_SSD_PARTITION"
lsblk -f "$VSLAM_SSD_PARTITION"
```

장치 전체에 ext4가 만들어져 있고 파티션이 없다면 실제 파일시스템 장치인 `/dev/nvmeXn1`을
지정한다. 이미 다른 경로에 마운트됐다면 먼저 그 경로의 사용 여부를 확인하고 마운트 경로를
하나로 정한다. 계속 사용하려면 이 가이드의 `/mnt/nova_ssd`를 그 실제 경로로 바꾼다.

### 2-B. 새 SSD가 완전히 비어 있을 때만 초기화한다

**이 블록은 선택한 디스크에 파티션을 만들고 파일시스템을 포맷한다. 데이터가 있는 SSD에는
실행하지 않는다.** 파일시스템/파티션 서명이 있거나 사용 중인 장치는 검사 단계에서 중단한다.

```bash
# 실제 새 SSD 장치명으로 바꾼 뒤, 이 블록 전체를 실행한다.
VSLAM_SSD_DEVICE=/dev/nvmeXn1
(
  set -euo pipefail
  test -b "$VSLAM_SSD_DEVICE"
  test "$(lsblk -dn -o TYPE "$VSLAM_SSD_DEVICE")" = disk
  lsblk -o NAME,PATH,SIZE,MODEL,FSTYPE,MOUNTPOINTS "$VSLAM_SSD_DEVICE"
  if lsblk -nr -o MOUNTPOINTS "$VSLAM_SSD_DEVICE" | grep -q '[^[:space:]]'; then
    echo '디스크 또는 하위 파티션이 사용 중입니다.' >&2
    exit 1
  fi
  if test -n "$(sudo wipefs --no-act --noheadings --output TYPE "$VSLAM_SSD_DEVICE")"; then
    echo '기존 디스크 서명이 있습니다. 초기화를 중단합니다.' >&2
    exit 1
  fi
  sudo apt-get update
  sudo apt-get install -y parted
  sudo parted --script "$VSLAM_SSD_DEVICE" mklabel gpt
  sudo parted --script "$VSLAM_SSD_DEVICE" mkpart primary ext4 1MiB 100%
  sudo partprobe "$VSLAM_SSD_DEVICE"
  sudo udevadm settle
  sudo mkfs.ext4 -L rby1_nvme "${VSLAM_SSD_DEVICE}p1"
)
```

위 블록이 성공한 뒤:

```bash
VSLAM_SSD_PARTITION="${VSLAM_SSD_DEVICE}p1"
```

### 2-C. 마운트와 재부팅 후 자동 마운트

```bash
test -b "$VSLAM_SSD_PARTITION"
sudo mkdir -p /mnt/nova_ssd
sudo mount "$VSLAM_SSD_PARTITION" /mnt/nova_ssd
findmnt /mnt/nova_ssd
df -h /mnt/nova_ssd
sudo blkid "$VSLAM_SSD_PARTITION"
```

`blkid` 출력의 실제 UUID를 사용한다. fstab을 백업하고 편집한다.

```bash
sudo cp -a /etc/fstab "/etc/fstab.rby1-backup-$(date +%Y%m%d-%H%M%S)"
sudoedit /etc/fstab
```

기존 항목을 보존하고, 동일 UUID/마운트 경로의 항목이 없으면 아래 한 줄을 추가한다.
`실제-SSD-UUID`를 출력된 UUID로 바꾼다.

```text
UUID=실제-SSD-UUID /mnt/nova_ssd ext4 defaults,nofail 0 2
```

```bash
sudo systemctl daemon-reload
sudo mount -a
mountpoint /mnt/nova_ssd
sudo chown "$(id -u):$(id -g)" /mnt/nova_ssd
mkdir -p /mnt/nova_ssd/workspaces/isaac_ros-dev/{src,maps,bags,docker}
touch /mnt/nova_ssd/workspaces/isaac_ros-dev/.write-check
rm /mnt/nova_ssd/workspaces/isaac_ros-dev/.write-check
```

가능한 시점에 U-PC를 재부팅한 뒤 `findmnt /mnt/nova_ssd`가 그대로 나오는지 확인한다.
이 SSD 배치 방식은 [NVIDIA SSD 가이드](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/jetson_storage.html)의
추가 저장소 방식에 해당한다.

## 3. Docker와 NVIDIA runtime을 준비한다 — 호스트

```bash
sudo apt-get update
sudo apt-get install -y git git-lfs curl ca-certificates rsync python3
git lfs install --skip-repo
docker --version
```

Docker가 없을 때만:

```bash
sudo apt-get install -y docker.io
sudo systemctl enable --now docker
```

현재 runtime을 확인한다.

```bash
sudo docker info --format '{{json .Runtimes}}'
command -v nvidia-ctk || true
dpkg-query -W nvidia-container-toolkit 2>/dev/null || true
```

출력에 `nvidia` runtime이 있고 이미 동작하면 기존 구성을 사용한다.
없으면 JetPack의 apt 환경에서 toolkit을 설치·등록한다.

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker --set-as-default
```

패키지를 찾을 수 없으면 [NVIDIA toolkit 저장소 설치](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian)를
먼저 적용한다. Jetson GPU 드라이버는 호스트 JetPack의 것을 사용한다.
Docker 재시작은 다음 SSD 저장소 변경까지 마친 뒤 수행한다.

```bash
sudo usermod -aG docker "$USER"
```

그룹 변경이 있었다면 로그아웃/로그인하고 새 호스트 터미널에서 계속한다.
`run_dev.sh`는 일반 사용자로 실행한다.

## 4. Docker 이미지 저장소도 SSD로 옮긴다 — 호스트

workspace만 SSD에 두면 이미지가 시스템 디스크를 차지할 수 있다.
현재 상태를 먼저 확인한다.

```bash
sudo docker info --format 'Root={{.DockerRootDir}} Driver={{.Driver}} Status={{json .DriverStatus}}'
sudo docker ps --format 'table {{.Names}}\t{{.Status}}'
```

아래 이전 명령은 **기존 `overlay2` 저장소**용이다. Docker의 containerd image store가
표시되면 `data-root`만 바꿔서는 이미지 저장소가 옮겨지지 않는다. 그 경우
[Docker 저장소 설정](https://docs.docker.com/engine/daemon/#daemon-data-directory)에 따라
containerd 저장소도 별도로 이전해야 한다. 이미 SSD의 root를 사용하면 복사 단계는 생략한다.

기존 컨테이너 작업을 종료한 뒤, 아래 블록 전체를 실행한다.
기존 저장소는 복구용으로 남긴다.

```bash
(
  set -euo pipefail
  mountpoint -q /mnt/nova_ssd
  VSLAM_OLD_DOCKER_ROOT=$(sudo docker info --format '{{.DockerRootDir}}')
  test "$(sudo docker info --format '{{.Driver}}')" = overlay2
  test -n "$VSLAM_OLD_DOCKER_ROOT"
  test "$VSLAM_OLD_DOCKER_ROOT" != /
  test "$VSLAM_OLD_DOCKER_ROOT" != /mnt/nova_ssd/docker
  test ! -e /mnt/nova_ssd/docker
  sudo systemctl stop docker.service docker.socket
  sudo mkdir -p /mnt/nova_ssd/docker
  sudo rsync -aHAX --numeric-ids "$VSLAM_OLD_DOCKER_ROOT/" /mnt/nova_ssd/docker/
  if test -f /etc/docker/daemon.json; then
    sudo cp -a /etc/docker/daemon.json "/etc/docker/daemon.json.rby1-backup-$(date +%Y%m%d-%H%M%S)"
  fi
  sudo python3 - <<'PY'
import json
from pathlib import Path
config = Path('/etc/docker/daemon.json')
settings = json.loads(config.read_text()) if config.exists() else {}
settings['data-root'] = '/mnt/nova_ssd/docker'
config.parent.mkdir(parents=True, exist_ok=True)
config.write_text(json.dumps(settings, indent=2) + '\n')
PY
  sudo dockerd --validate --config-file=/etc/docker/daemon.json
)
```

다음으로 SSD가 마운트된 뒤 Docker가 시작되도록 설정한다.

```bash
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo tee /etc/systemd/system/docker.service.d/10-rby1-ssd.conf >/dev/null <<'EOF'
[Unit]
RequiresMountsFor=/mnt/nova_ssd/docker
EOF
sudo systemctl daemon-reload
sudo systemctl restart docker
docker info --format 'Root={{.DockerRootDir}} Runtimes={{json .Runtimes}}'
docker ps -a
df -h /mnt/nova_ssd
```

Root가 `/mnt/nova_ssd/docker`이고 runtime에 `nvidia`가 표시되는지 확인한다.
이전 과정이 실패하면 먼저 원인을 확인한다. 저장소 변경을 되돌릴 때는 해당
`daemon.json.rby1-backup-*`을 원래 설정으로 복원하고 Docker를 재시작한다.
기존 Docker 데이터가 남아 있어야 원래 컨테이너/이미지를 다시 사용할 수 있다.

## 5. SSD에 workspace와 Isaac ROS 3.2 소스를 준비한다 — 호스트

새 호스트 터미널마다 아래 환경변수를 먼저 설정한다.

```bash
export ISAAC_ROS_WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
export ROS_DOMAIN_ID=0
mountpoint -q /mnt/nova_ssd
mkdir -p "$ISAAC_ROS_WS"/{src,maps,bags,docker}
docker ps
```

기존 로봇이 다른 domain을 사용하면 `0` 대신 그 값을 사용한다.
`docker ps`가 권한 오류 없이 실행돼야 한다.
`isaac_ros_common`이 아직 없는 첫 설치에서는:

```bash
git clone --branch release-3.2 \
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git \
  "$ISAAC_ROS_WS/src/isaac_ros_common"
git -C "$ISAAC_ROS_WS/src/isaac_ros_common" branch --show-current
```

이미 있으면 중복 clone 대신 현재 브랜치와 로컬 변경을 확인한다.
서로 다른 Isaac major 버전의 common/runtime을 섞지 않는다.
[3.2 개발 환경](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/dev_env_setup.html)

## 6. VSLAM 의존성을 포함한 영구 Docker 이미지를 만든다 — 호스트

3.2의 `run_dev.sh`는 `--rm` 컨테이너를 사용한다. 컨테이너 내부에서만 apt로 설치하면
컨테이너 종료 후 설치가 사라진다. 아래 custom layer에 VSLAM/Nav2/개발 도구를 포함한다.
[공식 image layer 구성](https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/docker_devenv/index.html#configuring-run-dev-sh)

```bash
cat > "$ISAAC_ROS_WS/docker/Dockerfile.rby1" <<'EOF'
ARG BASE_IMAGE
FROM ${BASE_IMAGE}
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-nav2-mppi-controller ros-humble-nav2-theta-star-planner \
    ros-humble-robot-state-publisher ros-humble-tf2-ros ros-humble-rviz2 \
    ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions python3-rosdep \
    python3-pyqt5 python3-yaml python3-pytest \
    && rm -rf /var/lib/apt/lists/*
EOF
```

먼저 기존 common 설정을 확인한다.

```bash
for VSLAM_CONFIG in \
  "$ISAAC_ROS_WS/src/isaac_ros_common/scripts/.isaac_ros_common-config" \
  "$HOME/.isaac_ros_common-config"; do
  if test -f "$VSLAM_CONFIG"; then
    printf '\n--- %s ---\n' "$VSLAM_CONFIG"
    cat "$VSLAM_CONFIG"
  fi
done
```

처음 만드는 설정 파일은 아래와 같이 작성한다. 기존 설정 파일은 백업하고 필요한 항목을
수정한다. 홈의 `.isaac_ros_common-config`가 있으면 local 파일보다 우선하므로 그 파일의
image key/search dirs/suffix도 이번 workspace에 맞아야 한다.

```bash
cat > "$ISAAC_ROS_WS/src/isaac_ros_common/scripts/.isaac_ros_common-config" <<'EOF'
CONFIG_IMAGE_KEY=ros2_humble.realsense.rby1
CONFIG_DOCKER_SEARCH_DIRS=("${ISAAC_ROS_WS}/docker")
CONFIG_CONTAINER_NAME_SUFFIX=rby1
EOF
```

D435i를 U-PC의 USB 3 포트에 연결한다. 해당 카메라를 사용하는 기존 ROS node/viewer는 종료한다.
일반 사용자로 이미지 빌드와 컨테이너 진입을 실행한다.

```bash
"$ISAAC_ROS_WS/src/isaac_ros_common/scripts/run_dev.sh" \
  -d "$ISAAC_ROS_WS" -i ros2_humble.realsense.rby1
```

이 명령이 끝나면 **컨테이너 내부 터미널**이다. 다음 사항을 확인한다.

```bash
echo "$ISAAC_ROS_WS"
echo "$ROS_DOMAIN_ID"
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
ros2 pkg prefix isaac_ros_visual_slam
ros2 pkg prefix realsense2_camera
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
for name, expected in [('isaac_ros_visual_slam', '3.2.'), ('realsense2_camera', '4.51.1')]:
    version = ET.parse(Path(get_package_share_directory(name)) / 'package.xml').getroot().findtext('version')
    print(f'{name}: {version}')
    if not version.startswith(expected):
        raise SystemExit(f'예상 버전이 아닙니다: {name} {version}')
PY
```

workspace는 `/workspaces/isaac_ros-dev`, ROS는 `humble`, Isaac은 `3.2.x`여야 한다.
이미지/라이브러리는 호스트의 `/mnt/nova_ssd/docker`, 소스/지도/bag은 SSD workspace에 저장된다.
RealSense layer는 librealsense 2.55.1과 realsense-ros `release/4.51.1-isaac`를 사용한다.
카메라 firmware의 공식 기준은 5.13.0.50이다.
[RealSense 설치 기준](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/sensors/realsense_setup.html)

## 7. 현재 `rby1_vslam` 코드를 SSD workspace에 넣는다

현재 이 채팅의 변경 파일은 Windows 로컬 `VSLAM` checkout에 있다.
아직 원격 Git에 반영되지 않은 파일은 `git clone --branch VSLAM`만으로 가져올 수 없다.
아래 두 방법 중 실제 코드가 있는 위치에 맞는 하나를 사용한다.

### 7-A. RBY1 호스트에 이번 코드가 이미 복사돼 있으면 — 호스트

```bash
export ISAAC_ROS_WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
VSLAM_SOURCE_REPO="$HOME/rby1_ros2_ws/src/rby1-project"
(
  set -euo pipefail
  test -f "$VSLAM_SOURCE_REPO/rby1_vslam/launch/isaac.launch.py"
  test ! -e "$ISAAC_ROS_WS/src/rby1_vslam"
  cp -a "$VSLAM_SOURCE_REPO/rby1_vslam" "$ISAAC_ROS_WS/src/rby1_vslam"
)
```

소스 저장소 경로가 다르면 `VSLAM_SOURCE_REPO`를 바꾼다.
대상 패키지가 이미 있으면 먼저 버전/변경 파일을 확인해 갱신한다.

### 7-B. 이번 Windows 작업 폴더에서 보내려면 — Windows PowerShell

RBY1 호스트에서 `whoami`, `hostname -I`로 로그인 사용자와 접속 가능한 IP를 확인한다.
아래 `사용자@JETSON_IP`를 실제 값으로 바꾸고 Windows에서 실행한다.

```powershell
$JetsonTarget = '사용자@JETSON_IP'
$VslamArchive = Join-Path $env:TEMP 'rby1_vslam_nvme.tar.gz'
tar -czf $VslamArchive --exclude=__pycache__ --exclude=.pytest_cache -C 'C:\Users\danw0\rby1-project' rby1_vslam
scp $VslamArchive "${JetsonTarget}:/mnt/nova_ssd/workspaces/isaac_ros-dev/rby1_vslam_nvme.tar.gz"
```

전송 후 **RBY1 호스트**에서:

```bash
export ISAAC_ROS_WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
(
  set -euo pipefail
  test ! -e "$ISAAC_ROS_WS/src/rby1_vslam"
  tar -xzf "$ISAAC_ROS_WS/rby1_vslam_nvme.tar.gz" -C "$ISAAC_ROS_WS/src"
  test -f "$ISAAC_ROS_WS/src/rby1_vslam/launch/isaac.launch.py"
)
```

## 8. VSLAM 패키지만 먼저 빌드한다 — 컨테이너

호스트에서 두 번째 터미널을 열고 아래 명령으로 같은 실행 중인 컨테이너에 들어간다.
컨테이너가 실행 중이면 재빌드 없이 새 shell이 열린다.

```bash
export ISAAC_ROS_WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
export ROS_DOMAIN_ID=0
"$ISAAC_ROS_WS/src/isaac_ros_common/scripts/run_dev.sh" \
  -d "$ISAAC_ROS_WS" -i ros2_humble.realsense.rby1
```

진입한 **컨테이너**에서:

```bash
source /opt/ros/humble/setup.bash
cd /workspaces/isaac_ros-dev
test -f /etc/ros/rosdep/sources.list.d/20-default.list || sudo rosdep init
rosdep update
rosdep install --from-paths src/rby1_vslam --ignore-src --rosdistro humble -y \
  --skip-keys 'isaac_ros_visual_slam isaac_ros_visual_slam_interfaces rby1_control'
colcon build --symlink-install --packages-select rby1_vslam
source install/setup.bash
ros2 pkg executables rby1_vslam
ros2 launch rby1_vslam isaac.launch.py --show-args
```

이 첫 단계는 카메라/cuVSLAM 확인용이다. waypoint/mobile UI는 기존 `rby1_control`의 Python
패키지도 필요하므로 이후 단계에서 준비한다. `rby1_control`을 건너뛰는 rosdep 옵션은 이
카메라 테스트를 위해 사용한다.

## 9. D435i와 GPU VSLAM을 확인한다 — 컨테이너 터미널 3개

각 터미널은 8단계의 호스트 `run_dev.sh` 명령으로 같은 컨테이너에 들어간다.
아래 source 명령은 컨테이너의 각 새 터미널마다 실행한다.

### 터미널 1: 카메라

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch rby1_vslam d435i.launch.py tracking_mode:=0
```

`RealSense Node Is Up!`와 IR 스트림 시작 로그를 확인한다.
여러 카메라가 있으면 `serial_no:=실제숫자시리얼`을 추가한다.

### 터미널 2: cuVSLAM

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch rby1_vslam isaac.launch.py tracking_mode:=0 start_rviz:=false
```

로그에 `cuVSLAM version`, GPU warm-up, tracker 초기화가 나타나는지 확인한다.
U-PC 데스크톱 화면을 사용할 때는 `start_rviz:=true`로 시각화할 수 있다.

### 터미널 3: 입력과 출력

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 topic hz /d435/d435/infra1/image_rect_raw
```

약 30 Hz인지 확인한 뒤 Ctrl-C로 해당 확인 명령을 끝내고 다음 명령을 실행한다.

```bash
ros2 topic hz /d435/d435/infra2/image_rect_raw
ros2 topic echo /visual_slam/status --once
ros2 topic hz /rby1/vslam/camera_odometry
ros2 topic echo /rby1/vslam/tracking_status --once
```

각 `topic hz`는 Ctrl-C로 종료한다. 정상 상태는 `vo_state: 1`과
`tracking_status`의 `ready: true`, `tracking_ok: true`다.
이 단계는 카메라 위치만 추적하므로 `vslam_map → base` TF는 아직 제공하지 않는다.
stereo+IMU를 시험할 때는 터미널 1과 2를 모두 종료하고 **둘 다** `tracking_mode:=1`로 다시
실행한다. 3.2에서 `tracking_mode:=2`는 VSLAM용으로 지원하지 않는다.

호스트에서 GPU 사용률을 볼 때는:

```bash
tegrastats --interval 1000
```

## 10. 기존 로봇 driver/control/TF와 연결한다

9단계의 카메라와 cuVSLAM launch를 종료한다. 기존 로봇 driver/control과 robot TF는
호스트에서 사용하던 방식으로 실행한다. 카메라는 컨테이너에서 시작하므로 호스트의 동일
D435i camera node는 종료한다.

컨테이너에서 호스트의 ROS 그래프가 보이는지 확인한다.

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 topic echo /rby1/odom --once
ros2 topic echo /rby1/joint_states --once
ros2 topic echo /rby1/control/state --once
ros2 run tf2_ros tf2_echo odom base
```

호스트/컨테이너의 `ROS_DOMAIN_ID`와 DDS 설정을 맞춘다. 호스트가 CycloneDDS를 쓰면
컨테이너에서도 실행 전에 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`를 지정한다.
호스트와 같은 설정이 필요한 다른 DDS 구성도 동일하게 적용한다.
공식 `run_dev.sh`는 host network와 host IPC를 사용한다.

먼저 GUI 없이 VSLAM과 Nav2만 연결할 수 있다.

```bash
ros2 launch rby1_vslam physical.launch.py tracking_mode:=0 \
  start_driver:=false start_description:=false start_control:=false \
  start_ui:=false
```

기존 D435 mount TF가 발행 중이면 `publish_mount_tf:=false`를 추가한다.
카메라를 이미 별도로 실행했다면 `start_camera:=false`를 추가하고 토픽과 optical TF를
확인한다. Nav2 gate가 `/rby1/cmd_raw`를 사용하므로 기존 odometry navigation 등 다른
`cmd_raw` 발행 노드는 종료한 상태에서 navigation을 사용한다.

```bash
ros2 topic echo /rby1/vslam/localization_status --once
ros2 topic echo /rby1/vslam/navigation_status --once
ros2 run tf2_ros tf2_echo vslam_map base
```

`localization_status.healthy=true`와 base TF가 나오면 베이스 위치 변환까지 연결된 것이다.
Nav2 gate는 시작 시 OFF다. `use_scan:=false`에는 장애물 감지·회피가 없다.
LiDAR가 실행 중이면 `use_scan:=true scan_topic:=/scan`을 추가한다.

### waypoint UI까지 준비하려면

기존 프로젝트와 vendor 메시지 패키지의 소스를 SSD workspace에 추가한다.
아래 경로는 기존 프로젝트에서 쓰던 경로 예시이며 실제 위치로 바꾼다.
이미 workspace에 패키지가 있으면 중복 복사하지 않는다.

**호스트:**

```bash
export ISAAC_ROS_WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
VSLAM_PROJECT_REPO="$HOME/rby1_ros2_ws/src/rby1-project"
VSLAM_VENDOR_REPO="$HOME/rby1_ros2_ws/src/rby1-ros2"
(
  set -euo pipefail
  test -f "$VSLAM_PROJECT_REPO/rby1_control/package.xml"
  test -f "$VSLAM_VENDOR_REPO/rby1_msgs/package.xml"
  test ! -e "$ISAAC_ROS_WS/src/rby1_control"
  test ! -e "$ISAAC_ROS_WS/src/rby1_msgs"
  cp -a "$VSLAM_PROJECT_REPO/rby1_control" "$ISAAC_ROS_WS/src/rby1_control"
  cp -a "$VSLAM_VENDOR_REPO/rby1_msgs" "$ISAAC_ROS_WS/src/rby1_msgs"
)
```

**컨테이너:**

```bash
source /opt/ros/humble/setup.bash
cd /workspaces/isaac_ros-dev
rosdep install --from-paths src/rby1_vslam src/rby1_control src/rby1_msgs \
  --ignore-src --rosdistro humble -y \
  --skip-keys 'isaac_ros_visual_slam isaac_ros_visual_slam_interfaces'
colcon build --symlink-install --packages-up-to rby1_vslam
source install/setup.bash
python3 -c 'import rby1_control.topic_protocol; print("control protocol import OK")'
```

위 작업은 기존 소스를 SSD workspace에서 빌드하는 절차다. 호스트의 기존 파일을 수정하지
않는다. UI는 호스트에서 실행 중인 control backend와 ROS 토픽으로 통신한다.
먼저 실행했던 GUI 없는 `physical.launch.py`를 종료하고 U-PC 데스크톱의 컨테이너에서:

```bash
ros2 launch rby1_vslam physical.launch.py tracking_mode:=0 \
  start_driver:=false start_description:=false start_control:=false \
  waypoints_file:=/workspaces/isaac_ros-dev/maps/waypoints.yaml
```

전원·서보·control manager·stream은 UI에서 켠다. 저장한 Point로 navigation을 요청하면
gate가 상태를 확인한 뒤 목표를 받는다. 전체 driver를 컨테이너에서 실행하는 구성은
vendor SDK/driver/description도 별도로 준비해야 하므로 첫 설치는 호스트 재사용 경로를
기준으로 한다.

## 11. 지도와 rosbag을 SSD에 저장한다 — 컨테이너

ROS와 workspace를 source한 새 컨테이너 터미널에서 실행한다.
mapping launch가 실행 중이어야 한다.

```bash
mkdir -p /workspaces/isaac_ros-dev/maps
ros2 service call /rby1/vslam/cancel std_srvs/srv/Trigger '{}'
ros2 run rby1_vslam map_tool save --path /workspaces/isaac_ros-dev/maps/rby1_01
ls -lh /workspaces/isaac_ros-dev/maps/rby1_01
```

`rby1_01`은 새 디렉터리 또는 빈 디렉터리여야 한다. 호스트에서는 같은 데이터가
`/mnt/nova_ssd/workspaces/isaac_ros-dev/maps/rby1_01`에 보인다.
`map_tool`은 3.2 완료 응답과 `.mdb` 파일 생성을 확인한다.

다음 세션에서는 mapping launch를 종료하고:

```bash
ros2 launch rby1_vslam physical.launch.py mode:=localization tracking_mode:=0 \
  map_path:=/workspaces/isaac_ros-dev/maps/rby1_01 \
  start_driver:=false start_description:=false start_control:=false \
  waypoints_file:=/workspaces/isaac_ros-dev/maps/waypoints.yaml
```

영상이 들어온 뒤 다른 컨테이너 터미널에서 위치 힌트를 전달한다.

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run rby1_vslam map_tool localize \
  --path /workspaces/isaac_ros-dev/maps/rby1_01 \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
```

힌트는 지도 좌표에서의 `d435_link` 위치/자세다. 0 값은 지도 생성 시작 자세에 돌아온
경우의 예시이며 실제 시작 위치가 다르면 힌트도 바꾼다. 지도와 같은 `tracking_mode`를 사용한다.
localization 모드는 저장 지도에서 위치가 확인돼야 navigation을 활성화할 수 있다.

센서 데이터를 기록하려면 같은 번호의 camera launch가 실행 중인 상태에서:

```bash
export RBY1_WORKSPACE_SETUP=/workspaces/isaac_ros-dev/install/setup.bash
export RBY1_BAG_DIR=/workspaces/isaac_ros-dev/bags
bash /workspaces/isaac_ros-dev/src/rby1_vslam/recorder.sh mode0
```

기록은 Ctrl-C로 종료한다. mode1 launch를 사용하면 recorder도 mode1을 사용한다.
stereo/IMU 비교와 재생 절차는 [패키지 README](../README.md)를 따른다.

## 12. 이후 매번 실행하는 명령

**호스트 새 터미널마다:**

```bash
export ISAAC_ROS_WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
export ROS_DOMAIN_ID=0
mountpoint -q /mnt/nova_ssd
"$ISAAC_ROS_WS/src/isaac_ros_common/scripts/run_dev.sh" \
  -d "$ISAAC_ROS_WS" -i ros2_humble.realsense.rby1 -b
```

`-b`는 이미 만들어 둔 이미지를 사용한다. Dockerfile/의존성을 변경했으면 `-b`를 빼고
이미지를 다시 빌드한다. 같은 명령으로 추가 터미널을 열 수 있다.
처음 컨테이너를 띄운 shell을 `exit`하면 컨테이너와 안의 ROS 프로세스가 모두 종료된다.

**컨테이너 새 터미널마다:**

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

이후 카메라 테스트 또는 `physical.launch.py` 중 필요한 명령을 실행한다.

## 막히는 지점별 확인

| 증상 | 먼저 볼 내용 |
|---|---|
| SSD가 안 보임 | `lsblk`, `findmnt /mnt/nova_ssd`, 장착/파티션/UUID |
| Docker 이미지가 시스템 디스크를 사용 | `DockerRootDir`, containerd image store 여부 |
| Docker permission denied | docker 그룹 반영 후 새 로그인, `docker ps` |
| `unknown runtime nvidia` | runtime 등록과 Docker 재시작 결과 |
| custom Dockerfile을 못 찾음 | search dirs, 홈 config의 우선순위 |
| VSLAM 버전 검사 실패 | common 브랜치와 설치된 Isaac 3.2.x 확인 |
| 카메라가 안 열림 | USB 3, 다른 camera/viewer 프로세스, 시리얼, firmware |
| 카메라 토픽은 있으나 tracking 안 됨 | 좌우 camera info/optical TF, 양쪽 tracking_mode |
| 카메라 odom은 있으나 base pose 없음 | 촬영 시점 `base → d435_link` TF와 joint states |
| 호스트 로봇 토픽이 안 보임 | domain/DDS 설정, 기존 노드 실행 여부 |
| gate를 켤 수 없음 | tracking/localization/wheel odom/다른 cmd_raw 발행기 |
| SSH에서 GUI 실행 실패 | U-PC 데스크톱 사용 또는 `start_ui:=false`/RViz OFF |

GPU·USB·DDS·실제 주행은 U-PC에서 확인한다. 이 문서는 해당 명령의 실행 결과를 대신하지 않는다.
