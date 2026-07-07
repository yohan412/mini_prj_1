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

### 수정된 파일

| 파일 | 변경 |
|------|------|
| `src/pinky_pro/pinky_bringup/setup.py` | `camera_stream_server` entry point 추가 |

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
| **5000** | `camera_stream_server` | 카메라 MJPEG 영상 |
| **8080** | `nav2_web_server` | SLAM/Nav2 웹 맵 UI |
| **8888** | Jupyter | 노트북 (카메라와 동시 사용 주의) |

---

## 3. 사전 준비

### 로봇

```bash
pip3 install flask opencv-python --break-system-packages
```

### 제어 PC

```bash
pip3 install ultralytics opencv-python --break-system-packages
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

- [ ] 로봇 `colcon build --packages-select pinky_bringup` 완료
- [ ] 로봇 `camera_stream_server` 실행
- [ ] 브라우저에서 `/video` 영상 확인
- [ ] PC `yolo_stream_viewer.py` 실행 → 검출 박스 표시
- [ ] `camera_publisher` / Jupyter 카메라 미실행
