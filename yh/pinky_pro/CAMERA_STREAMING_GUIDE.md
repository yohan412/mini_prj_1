# Pinky Pro 카메라 스트리밍 + YOLO 검출 가이드

로봇에서 카메라 영상을 ROS2 토픽으로 송출하고, 제어 PC에서 영상을 받아 YOLO 객체 검출을 수행하는 방법입니다.

---

## 1. 추가된 파일 위치

| 파일 | 위치 | 실행 위치 | 역할 |
|------|------|-----------|------|
| `camera_publisher.py` | `src/pinky_pro/pinky_bringup/pinky_bringup/` | **로봇** | pinkylib 카메라 → `/camera/image_raw` 토픽 송출 |
| `camera_publisher.launch.xml` | `src/pinky_pro/pinky_bringup/launch/` | **로봇** | 카메라 퍼블리셔만 단독 실행 |
| `bringup_robot_with_camera.launch.xml` | `src/pinky_pro/pinky_bringup/launch/` | **로봇** | 기존 bringup + 카메라 송출 동시 실행 |
| `yolo_ros_viewer.py` | `~/pinky_pro/` (프로젝트 루트) | **제어 PC** | ROS 토픽 수신 → YOLO 검출 → OpenCV 창 표시 |

### 수정된 기존 파일 (ROS 패키지 등록용)

| 파일 | 변경 내용 |
|------|-----------|
| `src/pinky_pro/pinky_bringup/setup.py` | `camera_publisher` 실행 항목 추가 |
| `src/pinky_pro/pinky_bringup/package.xml` | `sensor_msgs`, `cv_bridge` 의존성 추가 |

> `yolo_test.py`는 로봇에서 직접 카메라를 사용하는 기존 방식입니다.  
> 이번에 추가한 `yolo_ros_viewer.py`는 **제어 PC**에서 ROS 토픽으로 영상을 받는 방식입니다.

---

## 2. 사전 준비

### 로봇과 제어 PC 공통

- Ubuntu 24.04 + ROS2 Jazzy
- `~/pinky_pro` 워크스페이스에 pinky_pro 패키지 빌드 완료

### 네트워크 설정 (중요)

로봇과 PC가 **같은 WiFi**에 연결되어 있고, **ROS_DOMAIN_ID가 동일**해야 합니다.

1. PC를 로봇 WiFi에 연결  
   - SSID: `pinky_XXXX`  
   - 비밀번호: `pinkypro`
2. 로봇 IP: `192.168.4.1`
3. 로봇과 PC 모두 `~/.bashrc`에 동일한 값 설정:

```bash
export ROS_DOMAIN_ID=42   # 원하는 번호로 통일 (예: 42)
```

4. 설정 적용:

```bash
source ~/.bashrc
```

---

## 3. 빌드 (로봇 + PC 모두)

새 파일을 반영하려면 워크스페이스를 다시 빌드합니다.

```bash
cd ~/pinky_pro
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select pinky_bringup
source install/setup.bash
```

> 로봇에도 동일한 소스를 복사한 뒤, 로봇에서 위 빌드 명령을 실행해야 합니다.  
> (SSH: `ssh pinky@192.168.4.1`, 비밀번호: `1`)

---

## 4. 실행 방법

### 방법 A — bringup + 카메라 동시 실행 (권장)

**[로봇]**

```bash
cd ~/pinky_pro
source install/setup.bash
ros2 launch pinky_bringup bringup_robot_with_camera.launch.xml
```

로봇 구동(bringup)과 카메라 송출이 한 번에 시작됩니다.

### 방법 B — bringup과 카메라를 따로 실행

**[로봇] 터미널 1**

```bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

**[로봇] 터미널 2**

```bash
ros2 launch pinky_bringup camera_publisher.launch.xml
```

또는 노드만 실행:

```bash
ros2 run pinky_bringup camera_publisher
```

### FPS 조절 (선택)

기본 15fps입니다. 변경하려면:

```bash
ros2 launch pinky_bringup bringup_robot_with_camera.launch.xml camera_fps:=10.0
```

---

## 5. 제어 PC에서 영상 확인 및 YOLO 실행

### 5-1. 토픽 연결 확인

**[제어 PC]**

```bash
cd ~/pinky_pro
source install/setup.bash

# 토픽 목록 확인
ros2 topic list | grep camera

# 영상 수신 속도 확인 (Hz가 나오면 정상)
ros2 topic hz /camera/image_raw
```

영상만 미리 보려면:

```bash
ros2 run rqt_image_view rqt_image_view
```

### 5-2. YOLO 검출 창 실행

**[제어 PC]** — 사전에 ultralytics 설치 (최초 1회)

```bash
pip3 install ultralytics --break-system-packages
```

```bash
cd ~/pinky_pro
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 yolo_ros_viewer.py
```

- `Pinky YOLOv8 Detection (ROS)` 창에 검출 결과가 표시됩니다.
- 창을 닫거나 `q` 키를 누르면 프로그램이 종료됩니다.

### 필요한 YOLO 모델 파일 (프로젝트 루트)

`yolo_ros_viewer.py`와 같은 폴더(`~/pinky_pro/`)에 아래 파일이 있어야 합니다.

- `yolov8n.pt` (기본 모델)

다른 모델을 쓰려면 ROS 파라미터로 지정할 수 있습니다.

```bash
python3 yolo_ros_viewer.py --ros-args -p model_path:=/path/to/best.pt -p confidence:=0.45 -p iou:=0.5 -p imgsz:=960 -p max_det:=100
```

---

## 6. 전체 동작 흐름

```
[로봇]
  카메라 하드웨어
      ↓
  pinkylib.Camera
      ↓
  camera_publisher 노드
      ↓
  ROS2 토픽: /camera/image_raw
      ↓ (WiFi + ROS_DOMAIN_ID)
[제어 PC]
  yolo_ros_viewer.py (YOLOv8)
      ↓
  YOLOv8 검출 + OpenCV 창
```

---

## 7. 시뮬레이션(Gazebo)에서 사용

Gazebo는 이미 `/camera/image_raw`를 송출합니다. 로봇에서 `camera_publisher`를 실행할 필요 없이, PC에서 바로:

```bash
# Gazebo 실행 후
python3 ~/pinky_pro/yolo_ros_viewer.py
```

---

## 8. 트러블슈팅

| 증상 | 확인 사항 |
|------|-----------|
| PC에서 토픽이 안 보임 | `ROS_DOMAIN_ID` 로봇/PC 동일 여부, WiFi 연결 상태 |
| `ros2 topic hz`가 0 | 로봇에서 `camera_publisher` 실행 여부, pinkylib 카메라 동작 |
| `No module named pinkylib` (PC) | 정상 — pinkylib는 로봇 전용. PC에서는 `yolo_ros_viewer.py`만 실행 |
| `cv_bridge` / NumPy 오류 | `yolo_ros_viewer.py`는 cv_bridge 없이 동작함. venv 사용 시 `source ~/venv/ros/bin/activate` 후 실행 |
| `No module named ultralytics` | `pip3 install ultralytics --break-system-packages` |
| 영상은 오는데 YOLO 창이 안 뜸 | `yolov8n.pt`가 `~/pinky_pro/`에 있는지 확인 |
| 영상 끊김/지연 | `camera_fps:=10.0` 등으로 FPS 낮추기 |

### 연결 테스트 명령어

```bash
# 로봇에서 (카메라 노드 실행 중)
ros2 topic info /camera/image_raw

# PC에서
ros2 topic echo /camera/image_raw --no-arr | head
```

---

## 9. 요약 체크리스트

- [ ] 로봇/PC `ROS_DOMAIN_ID` 동일
- [ ] `colcon build --packages-select pinky_bringup` 완료
- [ ] 로봇: `bringup_robot_with_camera.launch.xml` 실행
- [ ] PC: `ros2 topic hz /camera/image_raw` 로 수신 확인
- [ ] PC: `python3 yolo_ros_viewer.py` 실행
