# LAB PC: Ubuntu 24.04 + RTX 5080 + driver 580 + Isaac ROS 4.5

2026-09-23에 release-4.5 공식 문서와 launch/API 소스를 확인했다. 권장 구성은 **기존 Humble 시뮬레이션 컨테이너 + 별도의 Isaac ROS CLI Jazzy 컨테이너**이다. 호스트 드라이버 580은 유지하며 cuVSLAM/CUDA 사용자 라이브러리는 새 컨테이너가 제공한다. 호스트에 Jazzy까지 따로 설치할 필요는 없다.

아래 명령은 LAB Linux에서 실행한다. 현재 패키지를 작성한 Windows 환경에서는 설치·GPU 실행을 하지 않았다.

## 1. 호스트 확인

```bash
lsb_release -ds
nvidia-smi
df -h "$HOME"
docker version
```

Isaac ROS 4.5 공식 x86 조건은 Ubuntu 24.04, ROS 2 Jazzy, CUDA 13.0 이상, NVIDIA driver 580 이상, Ampere 이상 GPU/8 GB 이상 GPU 메모리, 디스크 여유 32 GB 이상이다. 여러 이미지·지도·bag을 저장할 여유를 추가 확보한다. `nvidia-smi`의 CUDA 표시는 드라이버가 지원하는 상한이며 컨테이너 CUDA 설치 여부와는 다르다. [NVIDIA release-4.5 지원 환경](https://nvidia-isaac-ros.github.io/v/release-4.5/getting_started/index.html)

Docker가 이미 있으므로 설치를 반복하지 않는다. 없는 경우 [Docker 공식 Ubuntu 설치](https://docs.docker.com/engine/install/ubuntu/)를 따른다.

## 2. GPU passthrough 확인

이미 NVIDIA Container Toolkit이 동작하면 바로 마지막 GPU 확인 명령으로 넘어간다. 미설치인 경우:

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
```

설정 반영을 위한 `sudo systemctl restart docker`는 실행 중인 기존 시뮬레이션 컨테이너에 영향을 줄 수 있으므로 시뮬레이션을 종료한 시점에 수행한다. Docker 그룹 권한이 필요하면 `sudo usermod -aG docker "$USER"` 후 로그아웃/로그인한다.

```bash
docker run --rm --gpus all ubuntu:24.04 nvidia-smi
```

여기서 5080과 driver 580이 보여야 한다. 호스트에서만 보이고 컨테이너에서 안 보이면 VSLAM 설치 전에 runtime 설정부터 해결한다. [NVIDIA Container Toolkit 설치·Docker runtime 설정](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## 3. 4.5 저장소와 CLI

`release-4`나 `latest` 대신 `release-4.5` 저장소를 사용한다. 이미 Isaac 저장소가 있다면 중복 등록하지 말고 기존 파일을 확인한다.

```bash
sudo apt-get install -y curl gnupg software-properties-common
sudo add-apt-repository universe
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg
echo 'deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4.5 noble main' \
  | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list
sudo apt-get update
apt-cache policy isaac-ros-cli
sudo apt-get install -y isaac-ros-cli
sudo isaac-ros init docker

export ISAAC_ROS_WS="$HOME/workspaces/isaac_ros-dev"
mkdir -p "$ISAAC_ROS_WS/src"
isaac-ros activate
```

이제 **새 Jazzy 컨테이너 내부**이다. 이 workspace의 환경변수는 컨테이너에서 `/workspaces/isaac_ros-dev`로 바뀐다. 다음 터미널에서도 호스트에서 `export ISAAC_ROS_WS=...` 후 `isaac-ros activate`로 들어온다.

이 구조에서 RealSense는 UPC에 연결되므로 LAB 컨테이너에 `realsense` image layer나 USB 전달이 필요 없다. 이전 Isaac ROS 3.x의 `run_dev.sh` / `ros2_humble` image key 절차를 혼합하지 않는다. [4.5 CLI 초기화](https://nvidia-isaac-ros.github.io/v/release-4.5/getting_started/index.html#initialize-isaac-ros-cli), [4.5 workspace와 컨테이너 설정](https://nvidia-isaac-ros.github.io/v/release-4.5/concepts/dev_env/index.html)

## 4. cuVSLAM과 패키지 설치

LAB 호스트에서 이 저장소의 **`rby1_vslam` 폴더만** `$ISAAC_ROS_WS/src/rby1_vslam`로 복사한다. 컨테이너 내부에서는 `/workspaces/isaac_ros-dev/src/rby1_vslam`이다.

**컨테이너 내부**:

```bash
source /opt/ros/jazzy/setup.bash
nvidia-smi
sudo apt-get update
apt-cache policy ros-jazzy-isaac-ros-visual-slam
sudo apt-get install -y ros-jazzy-isaac-ros-visual-slam \
  python3-colcon-common-extensions python3-rosdep
cd /workspaces/isaac_ros-dev
rosdep install --from-paths src/rby1_vslam --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-select rby1_vslam
source install/setup.bash
ros2 pkg prefix isaac_ros_visual_slam
ros2 pkg executables rby1_vslam
```

`apt-cache policy`가 4.5가 아닌 다른 저장소를 가리키면 설치하기 전에 해당 컨테이너의 `/etc/apt/sources.list.d/`를 확인해 4.5로 맞춘다. 호스트와 컨테이너에 이미 다른 minor 버전 저장소가 함께 등록되어 있지 않도록 한다. [공식 cuVSLAM 바이너리 설치](https://nvidia-isaac-ros.github.io/v/release-4.5/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#build-isaac-ros-visual-slam)

컨테이너 안에서 `apt install`한 패키지는 그 컨테이너가 제거되면 사라질 수 있다. 일단 실험한 뒤 반복 사용 시 공식 [custom Docker image layer](https://nvidia-isaac-ros.github.io/v/release-4.5/concepts/dev_env/index.html#custom-docker-image-layers)에 `ros-jazzy-isaac-ros-visual-slam`과 이 패키지의 ROS 의존성을 넣는다. 소스·빌드·지도는 마운트된 workspace에 저장한다.

## 5. 포트와 ROS 영역

Humble 쪽 ROS domain과 Jazzy 쪽 ROS domain을 다르게 한다. 예를 들어 UPC/기존 로봇은 0, LAB VSLAM은 85이다. TCP에는 `ROS_DOMAIN_ID`가 적용되지 않는다.

LAB 호스트에서 컨테이너 네트워크를 확인한다:

```bash
docker ps --format 'table {{.Names}}\t{{.Ports}}'
docker inspect --format '{{.HostConfig.NetworkMode}}' YOUR_ISAAC_CONTAINER
```

`host`이면 UPC는 LAB 호스트 IP의 7447번으로 접근한다. bridge network를 사용하는 별도 컨테이너로 구성했다면 생성 시 `-p 7447:7447/tcp`를 설정해야 한다. CLI 설정은 설치된 `/usr/share/isaac-ros-cli/config.yaml`과 공식 개발환경 문서를 기준으로 조정한다. Linux host network가 현재 구성에서는 간단하다.

UFW가 켜져 있으면 UPC IP만 허용한다. 예를 들어 UPC가 `192.168.30.2`일 때:

```bash
sudo ufw allow from 192.168.30.2 to any port 7447 proto tcp
```

인터넷으로 포트를 전달하지 않는다. 현재 wire protocol은 실험실 LAN용이며 TLS/인증을 제공하지 않는다.

두 PC 모두 chrony/NTP로 시계를 맞춘다. 원본 timestamp를 그대로 전달하므로 시계 차이가 크면 UPC의 freshness 판정과 TF 조회가 실패한다. 서로 다른 PC끼리 wall time을 각각 발행하는 `/clock`은 사용하지 않는다.

## 6. GPU 첫 검증과 실행

NVIDIA [4.5 quickstart rosbag](https://nvidia-isaac-ros.github.io/v/release-4.5/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#download-quickstart-assets)을 먼저 실행하면 GPU/설치 문제와 TCP/카메라 문제를 분리할 수 있다. 공식 bag은 이 패키지의 D435i 토픽 구성과 다르므로 공식 quickstart launch와 remapping을 그대로 사용한다.

그 다음 LAB에서:

```bash
export ROS_DOMAIN_ID=85
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch rby1_vslam mapping.launch.py
```

UPC 실행과 측정, 지도 저장, navigation은 [실험 가이드](experiment.md)를 따른다.

## 호스트에 직접 Jazzy를 설치하려면

컨테이너 대신 native 실행도 4.5 문서에 안내되어 있다. 호스트에 Jazzy를 설치하고 [공식 4.5 Virtual Environment 절차](https://nvidia-isaac-ros.github.io/v/release-4.5/getting_started/index.html#initialize-isaac-ros-cli)를 따라 CUDA apt 저장소, NVIDIA apt 저장소, Jazzy 개발도구, Isaac rosdep을 설정한 뒤 `sudo isaac-ros init venv`와 `isaac-ros activate`를 사용한다. [ROS 2 Jazzy Ubuntu 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)도 함께 따른다.

`venv`는 Python만 분리하고 CUDA/OpenCV/TensorRT Debian 패키지는 호스트에 설치한다. 기존 실험 환경과의 충돌 가능성이 있으므로 현재 LAB에는 Docker 구성을 권장한다. native를 선택해도 UPC 통신과 패키지 launch는 동일하다.
