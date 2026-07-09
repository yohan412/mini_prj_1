# 멀티로봇 주방 오케스트레이션 가이드

목표는 **로봇 2대(A/B)**가 **서로 다른 맵**에서 동작하면서, 제어 PC에서 “요리 주문” 1건을 넣으면 각 로봇이 역할에 맞는 순서로 임무를 수행하도록 만드는 것입니다.

이 가이드는 기존 단일 로봇용 문서인 `FRUIT_NAV_QUEUE_GUIDE.md`와 분리해서 유지합니다.

---

## 1. 개요

- **로봇 A (supplier)**: 재료(사과/바나나/당근/젤리)로 인식된 클러스터로 이동 → A맵 교환장소로 이동
- **로봇 B (server)**: B맵 교환장소로 이동(동시에 진행 가능) → 요리(애플파이/당근스프/바나나푸딩/오렌지)로 인식된 클러스터로 이동 → 손님(`bell`)로 이동

---

## 2. 클래스 / 레시피(요리→재료) 매핑

### 전체 클래스(9개)

- 재료: `apple`, `banana`, `carrot`, `jelly`
- 요리: `apple_pie`, `carrot_soup`, `banana_pudding`, `orange`
- 손님: `bell`

### 요리→재료 매핑

- `apple_pie` → `apple`
- `carrot_soup` → `carrot`
- `banana_pudding` → `banana`
- `orange` → `jelly`

---

## 3. 실행 구조(브리지 방식)

### 로봇(각 1대당 1개)

- `robot_bridge.py`: 로봇 로컬에서 Nav2/scan/TF/초음파를 붙잡고 HTTP API로 제공
  - `GET /api/state`: pose, map, lidar_clusters, tracker_objects, queue 등
  - `POST /api/queue/add`: fruit/pose 명령 큐 추가
  - `POST /api/queue/stop_all`: 실행중+대기 모두 중지
  - `POST /api/labels/update`: PC에서 전달한 라벨을 로봇 tracker에 반영

### 제어 PC(1개)

- `yolo_nav_server.py --config kitchen_config.yaml`
  - 로봇별 `robot_session.py`를 띄워 카메라 스트림에 YOLO 수행
  - LiDAR 클러스터와 매칭하여 `/api/labels/update`로 로봇에 라벨을 전달
  - `recipe_orchestrator.py`가 주문 워크플로를 진행
  - `yolo_nav.html` UI에서 로봇 선택/주문/상태 확인

---

## 4. 포트 / 실행 위치

| 포트 | 프로그램 | 실행 위치 |
|---|---|---|
| 5000 | camera_stream_server (MJPEG) | 로봇 |
| 8080 | nav2_web_server (기존 Nav2 웹 UI) | 로봇 |
| 8091 | robot_bridge.py | 로봇 |
| 8090 | yolo_nav_server.py (kitchen 모드) | 제어 PC |

---

## 5. 설정 파일: `kitchen_config.yaml`

파일: `kitchen_config.yaml`

- 로봇 A/B의 `bridge_url`, `stream_url`
- 각 맵의 교환장소 좌표(`exchange_pose`)
- 레시피 매핑, 클래스 목록
- 오케스트레이터 타임아웃/폴링 주기
- 선택: 로봇별 `classify_zones` (맵 좌표 사각 영역). 비어 있으면 전체 맵에서 클래스 라벨링 허용, 구역을 넣으면 **그 안의 클러스터만** 클래스 투표/라벨 갱신

초기 값으로 `exchange_pose`는 (0,0,0)이라 실제 환경 좌표로 반드시 수정해야 합니다.

Fruit 미세 접근 순서:

1. **nav2** — 라벨 좌표 앞 `approach_distance`(기본 0.3m)까지 Nav2 이동  
2. **align** — LiDAR 목표 방향 정렬 + LiDAR/초음파 거리로 standoff 확인 (`approach_distance ± 0.05m`)  
3. **approach** — 초음파 ≤ `ultrasonic_stop_distance`(기본 0.10m)까지 저속 전진  

- align 타임아웃 기본 **10s** (거리 OK면 도착 처리, 아니면 approach 또는 Nav2 1회 재시도)  
- stall 타임아웃 기본 **10s** (LiDAR/초음파 거리 OK면 도착 처리)  
- fine approach `cmd_vel`은 **`/cmd_vel_nav`** 로 publish (Nav2 velocity_smoother 경유)

---

## 6. 실행 절차(권장)

### 실행 전 점검 (로봇 1대)

| # | 확인 | 명령 / 기대값 |
|---|------|----------------|
| 1 | bringup | LiDAR `/scan`, TF |
| 2 | Nav2 | AMCL pose, `:8080` (선택) |
| 3 | **초음파** | `ros2 topic info /us_sensor/range -v` → **Publisher: 1** (`pinky_sensor_adc`) |
| 4 | bridge | `ros2 node info /robot_bridge` → **Publishers: `/cmd_vel_nav`** |
| 5 | 코드 동기화 | §6.1 파일 복사 후 bridge 재시작 |

> `ros2 topic list`에 `/us_sensor/range`가 보여도 **Publisher 0**이면 센서 미실행입니다 (bridge 구독만 등록됨).

### 로봇 A / 로봇 B 각각

아래는 **로봇 1대 기준 5개 터미널**을 권장합니다. 로봇 A/B 모두 동일하게 실행하되, 각자 **맵 파일**과 **IP/포트**는 환경에 맞게 설정합니다.

> **중요:** `bringup_robot.launch.xml`에는 **초음파 ADC 노드가 포함되어 있지 않습니다.**  
> fruit 미세 접근(approach)을 쓰려면 **터미널 4 (`pinky_sensor_adc`)** 를 반드시 실행하세요.  
> 없으면 큐가 `Waiting for ultrasonic...` → `Approach timeout` 으로 실패합니다.

#### 터미널 1 — 로봇 bringup (LiDAR, 모터, TF 등)

```bash
cd ~/pinky_pro
source install/setup.bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

#### 터미널 2 — 카메라 HTTP 스트림 (MJPEG)

```bash
cd ~/pinky_pro
source install/setup.bash
ros2 launch pinky_bringup camera_stream.launch.xml
```

#### 터미널 3 — Nav2 + AMCL + map_server

로봇별로 맵이 다르면 `map:=...`를 각각 지정하세요.

```bash
cd ~/pinky_pro
source install/setup.bash
ros2 launch pinky_navigation web_nav2.launch.xml map:=/abs/path/to/<robot_map.yaml>
```

#### 터미널 4 — 초음파 센서 (필수, raspi/aarch64)

`pinky_sensor_adc`는 **ARM64(라즈베리 파이)에서만** 빌드됩니다. 최초 1회:

```bash
cd ~/pinky_pro
source install/setup.bash
colcon build --packages-select pinky_sensor_adc
source install/setup.bash
```

실행:

```bash
cd ~/pinky_pro
source install/setup.bash
ros2 run pinky_sensor_adc main_node
```

정상 확인:

```bash
ros2 topic info /us_sensor/range -v
# Publisher: pinky_sensor_adc, Subscription: robot_bridge

ros2 topic echo /us_sensor/range --field range
# 0.03 ~ 2.5 m 사이 값
```

I2C 실패 시 `Failed to init I2C communication.` — `/dev/i2c-1` 권한·wiringPi·배선 확인.

#### 터미널 5 — 로봇 브리지 (HTTP :8091)

PC에서 수정한 Python 파일을 **로봇에도 복사**한 뒤 실행하세요 (아래 §6.1 참고).

```bash
cd ~/pinky_pro
source install/setup.bash
python3 robot_bridge.py --port 8091
```

### 6.1 로봇 ↔ PC 코드 동기화

주방 모드에서 **fruit 큐·align·approach** 는 **로봇 `robot_bridge`** 에서 실행됩니다. PC만 갱신해도 approach 동작은 바뀌지 않습니다.

**로봇 A/B 각각** 아래 파일을 `~/pinky_pro/` 에 맞춰 두세요:

| 파일 | 역할 |
|------|------|
| `robot_bridge.py` | HTTP 브리지, Nav2, `/cmd_vel_nav` publish |
| `command_queue.py` | fruit/nav2/align/approach 큐 |
| `fruit_final_approach.py` | LiDAR·초음파 미세 접근 (command_queue가 import) |
| `lidar_object_tracker.py` | LiDAR 트래커 |
| `yolo_nav_fusion.py` | fruit goal 해석 |
| `classify_zones.py` | classify zone (선택) |

**제어 PC** (`8090`):

| 파일 | 역할 |
|------|------|
| `yolo_nav_server.py` | KitchenManager, UI API |
| `yolo_nav.html` | 웹 UI |
| `robot_session.py` | YOLO + bridge 폴링 |
| `kitchen_config.yaml` | 로봇 URL, 레시피 |

복사 후 **반드시 프로세스 재시작**:

```bash
# 로봇
pkill -f robot_bridge.py
cd ~/pinky_pro && source install/setup.bash
python3 robot_bridge.py --port 8091

# PC
python3 yolo_nav_server.py --config kitchen_config.yaml --port 8090
```

적용 확인 (로봇):

```bash
ros2 node info /robot_bridge | grep cmd_vel
# Publishers: /cmd_vel_nav  (직접 /cmd_vel 아님)

grep -n "is_fine_align_complete" ~/pinky_pro/fruit_final_approach.py
# 있으면 align/approach 최신 코드
```

### 제어 PC

제어 PC는 **로봇 2대를 동시에** 다루므로, `kitchen_config.yaml`에 아래를 먼저 채워두세요.

- `robots.robot_a.bridge_url`, `robots.robot_a.stream_url`
- `robots.robot_b.bridge_url`, `robots.robot_b.stream_url`
- `robots.robot_a.exchange_pose`, `robots.robot_b.exchange_pose` (각 로봇 맵 좌표 기준)

```bash
cd ~/pinky_pro
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
python3 yolo_nav_server.py --config kitchen_config.yaml --port 8090
```

---

## 7. UI 사용법(요약)

- Robot 패널의 Robot selector에서 `robot_a` / `robot_b`를 선택
- **Set Initial Pose** / **Set Pose (Queue)** 는 선택된 로봇 bridge로 전달 (`/api/robots/<id>/initialpose`, `/api/robots/<id>/queue/add`)
- Order 패널에서 요리를 클릭하면 주문이 시작됨
- `모든 명령 중지`는 주문 취소 + 양쪽 로봇 stop_all을 수행
- UI **초음파(m)** 가 `-` 이면 `pinky_sensor_adc` 미실행 가능성 큼

---

## 8. 파라미터 정합(중요)

Nav2의 `xy_goal_tolerance`와 큐의 `arrival-threshold`는 서로 맞춰야 “도착 후 다음 단계 진행”이 안정적입니다.

- 권장: `arrival_threshold >= xy_goal_tolerance`

Nav2 설정 파일: `src/pinky_pro/pinky_navigation/params/nav2_params.yaml`

### robot_bridge 주요 CLI (fruit 미세 접근)

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--approach-distance` | `0.3` | Nav2/align standoff (m) |
| `--align-timeout` | `10.0` | align 단계 최대 시간 (s) |
| `--stalled-timeout` | `10.0` | 움직임 없음 판정 (s) |
| `--approach-timeout` | `20.0` | approach 단계 최대 시간 (s) |
| `--ultrasonic-stop-distance` | `0.10` | 초음파 최종 정지 거리 (m) |
| `--approach-linear-speed` | `0.06` | approach 전진 속도 (m/s) |

---

## 9. 트러블슈팅

- **주문이 멈춤**: 로봇 한쪽 큐가 `failed/cancelled`인지 확인하고 `모든 명령 중지` 후 재시도
- **라벨이 안 붙음**: PC에서 카메라 스트림 연결/YOLO 모델 클래스(9개) 일치 여부 확인
- **교환장소에서 대기만 함**: `exchange_pose` 좌표가 잘못되었거나 한쪽 로봇이 도착 실패
- **`Waiting for ultrasonic` / `Approach timeout`**: `pinky_sensor_adc` 미실행. `ros2 topic info /us_sensor/range -v` → Publisher 0이면 §6 터미널 4 실행
- **큐 메시지에 `(d=..., us=..., tgt=...)` 없음**: 로봇 `command_queue.py`·`fruit_final_approach.py` 미동기화 또는 `robot_bridge` 미재시작
- **Set Pose OK, Initial Pose 405**: PC `yolo_nav_server.py` 구버전 — `/api/robots/<id>/initialpose` 포함본으로 재시작
- **PC `/api/state` 500**: 주방 모드에서 UI 초기화 직후 잠깐 발생 가능. `/api/robots/<id>/state` 사용
- **align/approach 시 전진 안 함**: `ros2 topic echo /cmd_vel_nav --field linear.x` 확인. bridge는 **`/cmd_vel_nav`** publish (velocity_smoother 경유)
- **LiDAR align stall / Retry Nav2**: AMCL yaw 미갱신·클러스터 오류. align 1회 실패 시 `Retry Nav2 to ...` 자동 재시도

### 초음파·cmd_vel 빠른 확인 (로봇)

```bash
ros2 topic info /us_sensor/range -v
ros2 node info /robot_bridge | grep -A2 Publishers
ros2 topic echo /cmd_vel_nav --field linear.x
```

### bridge state·큐 확인

```bash
curl -s http://127.0.0.1:8091/api/state | python3 -m json.tool | head -40
```

