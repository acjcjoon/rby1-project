# UPC 실행

UPC 데스크톱에서 아래 런처 중 하나를 실행한다.

| 용도 | 명령 |
|---|---|
| 베이스 수동 이동 | `bash run_mobile_base_upc.sh` |
| VSLAM + Nav2 + 웨이포인트 UI | `bash run_vslam_upc.sh lab_host:=<LAB-IP>` |

베이스 모드는 드라이버·제어·수동 Qt를 실행한다. VSLAM 모드는 여기에 카메라·TCP·
위치 추정·Nav2·웨이포인트 Qt·RViz를 포함한다. 전원·서보·control manager·stream은
작업자가 UI에서 켠다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install -y python3-pyqt5 ros-humble-nav2-mppi-controller
cd ~/rby1_ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-up-to \
  rby1_driver rby1_control rby1_vslam rby1_description
cd src/rby1-project
```

기본 workspace는 `~/rby1_ros2_ws`, UPC domain은 현재 환경 또는 `0`이다.
다른 설치 경로는 `RBY1_WORKSPACE_SETUP`, `RBY1_ROS_SETUP`으로 지정한다.
추가 인자는 ROS launch의 `name:=value` 형식이다.

```bash
bash run_mobile_base_upc.sh robot_address:=192.168.30.1:50051
```

## VSLAM과 네비게이션

LAB의 Isaac ROS 4.5/Jazzy 컨테이너에서 먼저 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=85
ros2 launch rby1_vslam lab.launch.py mode:=mapping
```

UPC에서 접근 가능한 LAB Wi-Fi IP를 넣는다.

```bash
bash run_vslam_upc.sh lab_host:=<LAB-IP>
```

IMU는 양쪽 기본 ON이다. 끌 경우 양쪽 launch에 `enable_imu:=false`를 지정한다.
UPC 런처는 `~/librealsense-rsusb-2.58.4/build-rsusb/Release`의 RSUSB librealsense를
사용한다. 설치 위치가 다르면 `RBY1_RSUSB_LIB_DIR`로 지정한다.

Qt에서 수동 이동 → Point 저장 → Point 선택 → `Navigate to Selected Point` 순서로 사용한다.
Nav2 속도 게이트는 처음에 OFF이며 유효한 센서·위치 상태에서 UI가 활성화한다.
기본 `use_scan:=false`에는 장애물 회피가 없다. LiDAR가 준비되면 `use_scan:=true`를 사용한다.
정렬된 occupancy map은 `occupancy_map:=/절대경로/map.yaml`로 지정한다.

지도 저장·재위치 추정·TCP 설정은 [VSLAM 실행 안내](rby1_vslam/README.md)를 따른다.
`run_planner_ui.sh`는 팔·그리퍼·시나리오를 포함하는 별도 planner 도구다.
