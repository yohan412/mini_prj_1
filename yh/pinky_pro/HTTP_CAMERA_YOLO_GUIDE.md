# HTTP 영상 스트리밍 + PC YOLO 검출 가이드

로봇에서 **HTTP MJPEG**로 영상을 송출하고, 제어 PC에서 **YOLOv8**로 객체를 검출하는 방식입니다.

ROS 영상 토픽(`camera_publisher`)은 **사용하지 않습니다.**

---

## 1. 추가된 파일

| 파일 | 위치 | 실행 위치 | 역할 |
|------|------|-----------|------|
| `camera_stream_server.py` | `src/pinky_pro/pinky_bringup/pinky_bringup/` | **로봇** | pinkylib → JPEG/MJPEG HTTP 송출 |
| `camera_stream.launch.xml` | `src/pinky_pro/pinky_bringup/launch/` | **로봇** | 영상 서버 launch |
| `yolo_stream_viewer.py` | `~/pinky_pro/` | **제어 PC** | HTTP 스트림 수신 + YOLO + OpenCV 창 |
| `yolo_nav_server.py` | `~/pinky_pro/` | **제어 PC** | HTTP YOLO + LiDAR 융합 + Nav2 큐 UI (`:8090`) |
| `command_queue.py` | `~/pinky_pro/` | (모듈) | FIFO 명령 큐 |
| `lidar_object_tracker.py` | `~/pinky_pro/` | (모듈) | LiDAR 클러스터 → map 좌표 |
| `yolo_nav_fusion.py` | `~/pinky_pro/` | (모듈) | YOLO + LiDAR 클래스 매칭 |
| `yolo_nav.html` | `~/pinky_pro/` | (UI) | 과일 내비 웹 UI |

### 수정된 파일

| 파일 | 변경 |
|------|------|
| `src/pinky_pro/pinky_bringup/setup.py` | `camera_stream_server` entry point 추가 |
| `camera_stream.launch.xml` | 기본 해상도 320×240 |
| `pinky_navigation/map/mini_prj_map.*` | 사전 매핑 맵 추가 |
| `web_nav2.launch.xml` | 기본 맵 `mini_prj_map.yaml` |

### 사용하지 않는 파일 (참고)

| 파일 | 비고 |
|------|------|
| `camera_publisher.py` | ROS 토픽 방식 — 이번 방식에서 미사용 |
| `yolo_ros_viewer.py` | ROS 토픽 방식 — 이번 방식에서 미사용 |
| `nav2_web_server.py` | 맵 웹 UI (`:8080`) — 수정 없음 |

---

## 2. 포트 구분

| 포트 | 프로그램 | 용도 |
|------|----------|------|
| **5000** | `camera_stream_server` | 카메라 MJPEG 영상 (기본 320×240) |
| **8080** | `nav2_web_server` | SLAM/Nav2 웹 맵 UI |
| **8090** | `yolo_nav_server` | 과일 내비 + 명령 큐 웹 UI |
| **8888** | Jupyter | 노트북 (카메라와 동시 사용 주의) |

---

## 3. 사전 준비

### 로봇

```bash
pip3 install flask opencv-python --break-system-packages
```

### 제어 PC

```bash
pip3 install ultralytics opencv-python flask --break-system-packages
```

### 빌드 (로봇)

```bash
cd ~/pinky_pro
colcon build --packages-select pinky_bringup
source install/setup.bash
```

---

## 4. 실행 방법

### 4-1. 로봇 — 영상 서버

```bash
cd ~/pinky_pro
source install/setup.bash
ros2 run pinky_bringup camera_stream_server
```

또는 launch:

```bash
ros2 launch pinky_bringup camera_stream.launch.xml width:=320 height:=240
```

기본값이 320×240이므로 인자 생략 가능:

```bash
ros2 launch pinky_bringup camera_stream.launch.xml
```

옵션 예시:

```bash
ros2 launch pinky_bringup camera_stream.launch.xml fps:=10.0 jpeg_quality:=80
```

직접 실행:

```bash
ros2 run pinky_bringup camera_stream_server -- --port 5000 --fps 15 --jpeg-quality 75
```

### 4-2. 영상 확인 (브라우저)

```
http://192.168.4.1:5000/video
http://192.168.4.1:5000/snapshot
http://192.168.4.1:5000/health
```

### 4-3. 제어 PC — YOLO 검출

```bash
cd ~/pinky_pro
python3 yolo_stream_viewer.py --url http://192.168.4.1:5000/video
```

학습 모델 사용:

```bash
python3 yolo_stream_viewer.py \
  --url http://192.168.4.1:5000/video \
  --model ~/pinky_pro/best.pt \
  --confidence 0.45 \
  --imgsz 640
```

- 검출 클래스: **apple, banana, orange, carrot**
- 창 닫기 또는 `q` 키로 종료

### 4-4. 제어 PC — 과일 내비게이션 + 명령 큐 UI (`:8090`)

사전 매핑 맵 `mini_prj_map` + AMCL + LiDAR 객체 추적 + YOLO 클래스 라벨링.

```bash
# [로봇]
ros2 launch pinky_bringup bringup_robot.launch.xml
ros2 launch pinky_bringup camera_stream.launch.xml
source ~/pinky_pro/install/setup.bash
ros2 launch pinky_navigation web_nav2.launch.xml

# [PC] — ROS_DOMAIN_ID를 로봇과 동일하게
export ROS_DOMAIN_ID=<로봇과_동일>
source ~/pinky_pro/install/setup.bash
python3 ~/pinky_pro/yolo_nav_server.py \
  --stream-url http://192.168.4.1:5000/video \
  --model ~/pinky_pro/best.pt \
  --imgsz 320 \
  --port 8090
```

브라우저: `http://localhost:8090`

**상세 가이드:** [FRUIT_NAV_QUEUE_GUIDE.md](FRUIT_NAV_QUEUE_GUIDE.md) (빌드, 파일 목록, API, 트러블슈팅)

**UI 기능:**
- 맵(`mini_prj_map`) + 감지된 과일 마커 표시
- YOLO 영상 스트림 (320×240)
- 과일 버튼 → 명령 큐 추가 (FIFO 순차 실행)
- 처음 위치 저장 / 처음 위치로 이동 (큐)
- 맵 클릭+드래그 → pose 큐 추가
- Stop / Clear Queue

**LiDAR + YOLO 융합:**
1. `/scan` 클러스터 → `mini_prj_map` free 영역만 동적 장애물로 추정
2. map 좌표로 `object_registry` 저장
3. YOLO bbox 방위각과 LiDAR 클러스터 매칭 → 클래스 라벨
4. 큐 `fruit:apple` → registry map 좌표로 `navigate_to_pose`

**주의:** `:8090` UI 사용 중에는 `:8080` 맵 goal과 동시 사용하지 마세요.

---

## 5. 동작 흐름

```
[로봇]
  pinkylib.Camera
      ↓
  camera_stream_server (JPEG/MJPEG)
      ↓ HTTP :5000/video
[제어 PC]
  yolo_stream_viewer.py
      ↓
  YOLOv8 검출 + OpenCV 창
```

Nav2/SLAM 웹(`:8080`)과 영상 서버(`:5000`)는 **독립**으로 동작합니다.

---

## 6. 트러블슈팅

| 증상 | 확인 사항 |
|------|-----------|
| `Device or resource busy` | `camera_publisher`, Jupyter 카메라 노트북, 다른 카메라 프로세스 종료 |
| PC에서 스트림 안 열림 | 로봇 IP, WiFi 연결, `curl http://192.168.4.1:5000/health` |
| YOLO 클래스 오류 | `best.pt`에 apple/banana/orange/carrot 클래스 포함 여부 |
| 영상 지연 | `fps:=10`, PC에서 버퍼 비우기 로직 내장 (`grab` 반복) |
| 화질 부족 | `jpeg_quality:=80` 또는 `85`로 상향 |

### 카메라 점유 확인 (로봇)

```bash
# 다른 카메라 사용 프로세스가 없어야 함
ps aux | grep -E 'camera|jupyter|picam'
```

---

## 7. bringup과 함께 쓸 때

```bash
# 터미널 1 — 주행
ros2 launch pinky_bringup bringup_robot.launch.xml

# 터미널 2 — 영상 서버
ros2 launch pinky_bringup camera_stream.launch.xml

# 터미널 3 (선택) — SLAM 웹
ros2 launch pinky_navigation web_slam.launch.xml
```

PC:

```bash
python3 ~/pinky_pro/yolo_stream_viewer.py --url http://192.168.4.1:5000/video
```

---

## 8. 체크리스트

- [ ] 로봇 `colcon build --packages-select pinky_bringup pinky_navigation` 완료
- [ ] 로봇 `camera_stream_server` 실행 (320×240)
- [ ] 로봇 `web_nav2.launch.xml` + `mini_prj_map` 실행
- [ ] 브라우저에서 `/video` 영상 확인
- [ ] PC `yolo_stream_viewer.py` 또는 `yolo_nav_server.py` 실행
- [ ] `:8090` UI에서 과일 큐 + 맵 객체 마커 확인
- [ ] `camera_publisher` / Jupyter 카메라 미실행
