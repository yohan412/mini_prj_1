# 과일 내비게이션 + 명령 큐 웹 UI 가이드

사전 매핑 맵(`mini_prj_map`) 위에서 LiDAR로 동적 장애물(과일) 좌표를 추정하고, YOLO로 클래스를 라벨링한 뒤, PC 웹 UI에서 명령 큐로 Nav2 주행을 제어하는 기능입니다.

멀티로봇 주방 오케스트레이션(2대 로봇, 별도 맵, 요리 주문 워크플로)은 별도 문서인 `KITCHEN_MULTI_ROBOT_GUIDE.md`를 참고하세요.

관련 계획: `.cursor/plans/과일_내비_큐_ui_14c7cd2c.plan.md`

---

## 1. 개요

| 항목 | 내용 |
|------|------|
| 맵 | 사전 매핑 `mini_prj_map` (SLAM 아님, AMCL 로컬라이제이션) |
| 위치 추정 | LiDAR `/scan` 클러스터 → map 좌표 (`lidar_object_tracker.py`) |
| 클래스 라벨 | HTTP 320×240 영상 + YOLO (`yolo_nav_fusion.py`) |
| 주행 | Nav2 `navigate_to_pose` (PC에서 액션 클라이언트) |
| UI | Flask 웹 서버 `:8090` (`yolo_nav_server.py` + `yolo_nav.html`) |
| 명령 큐 | fruit / home / pose FIFO 순차 실행 (`command_queue.py`) |

### 포트 구분

| 포트 | 프로그램 | 실행 위치 |
|------|----------|-----------|
| 5000 | `camera_stream_server` | 로봇 — MJPEG 영상 (320×240) |
| 8080 | `nav2_web_server` | 로봇 — 기존 Nav2/SLAM 웹 UI |
| **8090** | `yolo_nav_server` | **PC** — 과일 내비 + 큐 UI |

> `:8090` 사용 중에는 `:8080`에서 맵 goal을 동시에 보내지 마세요. Nav2 goal이 충돌할 수 있습니다.

---

## 2. 생성된 파일

프로젝트 루트 (`~/pinky_pro/`)

| 파일 | 역할 |
|------|------|
| `yolo_nav_server.py` | PC 메인 서버 — Flask API, ROS2 브리지, YOLO 스레드, 큐 워커, Nav2 액션 |
| `yolo_nav.html` | 웹 UI — 맵, YOLO 영상, 과일 버튼, Home, 명령 큐 |
| `command_queue.py` | FIFO 명령 큐 (fruit / home / pose) |
| `lidar_object_tracker.py` | LiDAR 클러스터링, 정적맵 필터, map 좌표 object registry |
| `yolo_nav_fusion.py` | YOLO bbox + LiDAR 클러스터 방위각 매칭, goal 좌표 계산 |
| `FRUIT_NAV_QUEUE_GUIDE.md` | 본 문서 |

맵 패키지 (`src/pinky_pro/pinky_navigation/map/`)

| 파일 | 역할 |
|------|------|
| `mini_prj_map.pgm` | 사전 매핑 맵 이미지 |
| `mini_prj_map.yaml` | 맵 메타데이터 (resolution 0.05m, origin [-0.372, -0.638, 0]) |

---

## 3. 수정된 파일

| 파일 | 변경 내용 |
|------|-----------|
| `src/pinky_pro/pinky_bringup/launch/camera_stream.launch.xml` | 기본 해상도 `width:=320`, `height:=240` |
| `src/pinky_pro/pinky_bringup/pinky_bringup/camera_stream_server.py` | `DEFAULT_WIDTH=320`, `DEFAULT_HEIGHT=240` |
| `src/pinky_pro/pinky_navigation/launch/web_nav2.launch.xml` | 기본 맵 `mini_prj_map.yaml` |
| `HTTP_CAMERA_YOLO_GUIDE.md` | 8090 서버, mini_prj_map, LiDAR 융합 섹션 추가 |

### 변경하지 않은 파일

| 파일 | 비고 |
|------|------|
| `nav2_web_server.py` / `index.html` | 8080 기존 Nav2 웹 UI 유지 |
| `nav2_params.yaml` | Nav2 파라미터 변경 없음 |
| `pinky_interfaces` | 신규 ROS 메시지 추가 없음 (JSON API 사용) |

---

## 4. 아키텍처

```
[로봇 192.168.4.1]
  bringup_robot        → /scan, /odom, TF
  camera_stream_server → HTTP :5000/video (320×240)
  web_nav2.launch      → map_server(mini_prj_map) + AMCL + Nav2

[제어 PC]
  yolo_nav_server :8090
    ├─ HTTP 수신 → YOLO 검출
    ├─ ROS /scan  → LiDAR 클러스터 → map 좌표
    ├─ ROS /map   → 정적맵 필터 (free 셀만 동적 장애물)
    ├─ YOLO + LiDAR 매칭 → object_registry (class, map_x, map_y)
    ├─ command_queue → navigate_to_pose
    └─ yolo_nav.html (브라우저 UI)
```

### LiDAR + YOLO 융합 요약

1. **LiDAR**: `/scan` 포인트 클러스터링 → TF로 map 좌표 변환
2. **정적맵 필터**: `mini_prj_map`에서 **free(0)** 셀에 있는 hit만 동적 장애물(과일) 후보
3. **YOLO**: bbox 중심 → 카메라 방위각 → LiDAR 클러스터와 매칭 → 클래스 라벨
4. **내비**: 큐 `fruit:apple` → registry에서 apple map 좌표 → 접근 오프셋 0.5m goal 전송

---

## 5. 빌드 가이드

### 5-1. 로봇

```bash
cd ~/pinky_pro

# 의존성 (최초 1회)
pip3 install flask opencv-python --break-system-packages

# 맵·launch 반영을 위해 pinky_navigation 빌드 필수
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select pinky_bringup pinky_navigation
source install/setup.bash
```

빌드 후 맵 설치 경로 확인:

```bash
ros2 pkg prefix pinky_navigation
# → .../install/pinky_navigation/share/pinky_navigation/map/mini_prj_map.yaml
```

### 5-2. 제어 PC

```bash
cd ~/pinky_pro

# Python 패키지 (최초 1회)
pip3 install ultralytics opencv-python flask --break-system-packages

# ROS2 Jazzy + pinky_pro 워크스페이스 source
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash

# 로봇과 동일한 ROS 도메인
export ROS_DOMAIN_ID=<로봇과_동일>   # 예: 42
```

YOLO 모델 (`best.pt`)가 프로젝트 루트에 있어야 합니다. 클래스: **apple, banana, orange, carrot**.

---

## 6. 실행 가이드

### 6-1. 로봇 (3터미널)

```bash
# 터미널 1 — 로봇 bringup (LiDAR, 모터, TF)
source ~/pinky_pro/install/setup.bash
ros2 launch pinky_bringup bringup_robot.launch.xml

# 터미널 2 — 카메라 HTTP 스트림 (320×240)
source ~/pinky_pro/install/setup.bash
ros2 launch pinky_bringup camera_stream.launch.xml

# 터미널 3 — Nav2 + mini_prj_map (8080 웹 UI 포함, 선택)
source ~/pinky_pro/install/setup.bash
ros2 launch pinky_navigation web_nav2.launch.xml
```

`web_nav2.launch.xml`은 기본값으로 `mini_prj_map.yaml`을 사용합니다. 명시적으로 지정하려면:

```bash
ros2 launch pinky_navigation web_nav2.launch.xml \
  map:=$(ros2 pkg prefix pinky_navigation)/share/pinky_navigation/map/mini_prj_map.yaml
```

### 6-2. 제어 PC

```bash
export ROS_DOMAIN_ID=<로봇과_동일>
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash

cd ~/pinky_pro
python3 yolo_nav_server.py \
  --stream-url http://192.168.4.1:5000/video \
  --model ~/pinky_pro/best.pt \
  --imgsz 320 \
  --port 8090
```

브라우저: **http://localhost:8090**

### 6-3. 초기 설정 순서 (권장)

1. 로봇 bringup + Nav2 + 카메라 스트림 실행
2. PC에서 `ros2 topic echo /scan --once` 로 LiDAR 수신 확인
3. 브라우저 `:8090` → 맵에서 **Set Initial Pose** 로 AMCL 초기 위치 설정
4. **현재 위치를 처음 위치로 저장** (Home)
5. 과일이 보이면 맵에 객체 마커 표시 확인
6. 과일 버튼으로 큐에 명령 추가

---

## 7. 웹 UI 사용법

### 왼쪽 — 맵

- `mini_prj_map` + 로봇 pose + 경로 + **감지된 과일 마커** (클래스별 색상)
- 클릭+드래그: **Set Pose (Queue)** — pose 명령을 큐에 추가
- 클릭+드래그: **Set Initial Pose** — AMCL 초기 위치

### 오른쪽 패널

| 패널 | 기능 |
|------|------|
| Robot | pose, 주행 상태, Stop |
| Camera + YOLO | MJPEG 영상 + 검출 박스 |
| Fruit Commands | apple/banana/orange/carrot → 큐 추가 |
| Home | 처음 위치 저장 / 처음 위치로 이동 (큐) |
| Command Queue | pending/running/completed/failed 목록, Clear Pending |

### 명령 큐 규칙

- **FIFO** — 한 번에 하나씩 순차 실행
- `fruit`: registry에서 해당 클래스 map 좌표로 이동 (30s 내 미검출 시 failed)
- `home`: 저장된 Home pose로 이동
- `pose`: 맵에서 지정한 좌표로 이동
- 동일 클래스가 여러 개면 **로봇에 가장 가까운** 객체 선택

---

## 8. REST API

| 엔드포인트 | 메서드 | 설명 |
|-----------|--------|------|
| `/` | GET | `yolo_nav.html` |
| `/api/state` | GET | 맵, pose, queue, home, detections, map_objects |
| `/api/video_feed` | GET | YOLO 오버레이 MJPEG |
| `/api/queue/add` | POST | `{"type":"fruit","params":{"class":"apple"}}` |
| `/api/queue/remove` | POST | `{"id":"<command_id>"}` (pending만) |
| `/api/queue/clear` | POST | pending 전체 삭제 |
| `/api/home/set` | POST | 현재 pose를 Home으로 저장 |
| `/api/home/go` | POST | Home 이동 큐 추가 |
| `/api/nav/stop` | POST | Nav2 정지 + running 명령 cancelled |
| `/api/initialpose` | POST | `{"x", "y", "yaw"}` AMCL 초기 위치 |
| `/api/goal` | POST | `{"x", "y", "yaw"}` pose 큐 추가 |

Home pose 저장 경로: `~/.pinky_pro/home_pose.json`

---

## 9. 주요 파라미터

`yolo_nav_server.py` CLI 옵션:

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--stream-url` | `http://192.168.4.1:5000/video` | 로봇 MJPEG URL |
| `--model` | `best.pt` | YOLOv8 모델 경로 |
| `--port` | `8090` | Flask 포트 |
| `--imgsz` | `320` | YOLO 입력 크기 |
| `--confidence` | `0.2` | YOLO confidence |
| `--hfov-deg` | `66.0` | 카메라 수평 FOV |
| `--approach-distance` | `0.5` | 과일 앞 정지 거리 (m) |
| `--arrival-threshold` | `0.3` | 도착 판정 거리 (m) |
| `--detection-timeout` | `30.0` | fruit 명령 검출 대기 (s) |
| `--goal-update-interval` | `2.0` | fruit goal 갱신 주기 (s) |
| `--cluster-angle-tol` | `3.0` | LiDAR 클러스터 각도 (deg) |
| `--cluster-dist-tol` | `0.15` | LiDAR 클러스터 거리 (m) |
| `--object-match-angle` | `10.0` | YOLO-LiDAR 매칭 각도 (deg) |
| `--object-ttl` | `10.0` | 미감지 객체 유지 시간 (s) |
| `--map-match-tolerance` | `0.2` | 동일 객체 판정 거리 (m) |

---

## 10. 트러블슈팅

| 증상 | 확인 사항 |
|------|-----------|
| 맵이 안 보임 | `web_nav2.launch.xml` 실행, `colcon build pinky_navigation` 후 source |
| `/scan` 없음 | `bringup_robot.launch.xml`, WiFi, `ROS_DOMAIN_ID` 일치 |
| YOLO 영상 없음 | `curl http://192.168.4.1:5000/health`, 카메라 점유 프로세스 확인 |
| 과일 마커 없음 | AMCL initial pose 설정, LiDAR가 과일 높이에서 hit 하는지 확인 |
| fruit 명령 failed | 30s 내 YOLO+LiDAR 매칭 실패 — 과일이 카메라/LiDAR 시야에 있는지 확인 |
| goal 충돌 | 8080과 8090 동시 goal 사용 금지 |
| `best.pt` 없음 | 학습 모델 배치 또는 `--model yolov8n.pt` (클래스 불일치 주의) |
| 빌드 후 맵 못 찾음 | `install/pinky_navigation/share/pinky_navigation/map/` 에 pgm/yaml 존재 확인 |

### 카메라 점유 확인 (로봇)

```bash
ps aux | grep -E 'camera|jupyter|picam'
```

### ROS 연결 확인 (PC)

```bash
export ROS_DOMAIN_ID=<로봇과_동일>
ros2 topic list | grep -E 'scan|map|plan'
ros2 topic echo /scan --once
```

---

## 11. 체크리스트

- [ ] `colcon build --packages-select pinky_bringup pinky_navigation` 완료
- [ ] `install/.../map/mini_prj_map.yaml` 존재
- [ ] 로봇 `bringup_robot` + `camera_stream` + `web_nav2` 실행
- [ ] PC `ROS_DOMAIN_ID` 로봇과 동일
- [ ] `best.pt` 준비 (apple/banana/orange/carrot)
- [ ] `yolo_nav_server.py` 실행 → `:8090` 접속
- [ ] AMCL initial pose 설정
- [ ] Home 위치 저장
- [ ] 과일 마커 + 큐 주행 테스트

---

## 12. 관련 문서

- [HTTP_CAMERA_YOLO_GUIDE.md](HTTP_CAMERA_YOLO_GUIDE.md) — HTTP 영상 스트리밍 + YOLO 기본 가이드
- [CAMERA_STREAMING_GUIDE.md](CAMERA_STREAMING_GUIDE.md) — ROS 토픽 방식 (본 기능에서는 미사용)
