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

Fruit 미세 접근 순서: Nav2로 라벨 좌표 근처 도착 → **LiDAR 방위각 정렬** → **초음파 전진**. 근거리에서는 카메라 YOLO 정렬을 쓰지 않습니다.

---

## 6. 실행 절차(권장)

### 로봇 A / 로봇 B 각각

아래는 **로봇 1대 기준 4개 터미널**을 권장합니다. 로봇 A/B 모두 동일하게 실행하되, 각자 **맵 파일**과 **IP/포트**는 환경에 맞게 설정합니다.

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

#### 터미널 4 — 로봇 브리지 (HTTP :8091)

```bash
cd ~/pinky_pro
source install/setup.bash
python3 robot_bridge.py --port 8091
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
- Order 패널에서 요리를 클릭하면 주문이 시작됨
- `모든 명령 중지`는 주문 취소 + 양쪽 로봇 stop_all을 수행

---

## 8. 파라미터 정합(중요)

Nav2의 `xy_goal_tolerance`와 큐의 `arrival-threshold`는 서로 맞춰야 “도착 후 다음 단계 진행”이 안정적입니다.

- 권장: `arrival_threshold >= xy_goal_tolerance`

Nav2 설정 파일: `src/pinky_pro/pinky_navigation/params/nav2_params.yaml`

---

## 9. 트러블슈팅

- **주문이 멈춤**: 로봇 한쪽 큐가 `failed/cancelled`인지 확인하고 `모든 명령 중지` 후 재시도
- **라벨이 안 붙음**: PC에서 카메라 스트림 연결/YOLO 모델 클래스(9개) 일치 여부 확인
- **교환장소에서 대기만 함**: `exchange_pose` 좌표가 잘못되었거나 한쪽 로봇이 도착 실패

