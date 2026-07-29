#!/usr/bin/env python3
"""
Control-PC side robot session.

One RobotSession corresponds to one robot-side RobotBridge instance.

Responsibilities:
- Poll RobotBridge /api/state (pose + lidar_clusters) for fusion context.
- Read MJPEG stream from robot camera (HTTP) and run YOLO locally.
- Match YOLO detections to LiDAR clusters and push label updates to RobotBridge
  via /api/labels/update so the robot can run fruit-class navigation locally.
- Provide latest JPEG frame for the PC UI.
"""

from __future__ import annotations

import json
import math
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import numpy as np
from ultralytics import YOLO

from classify_zones import ClassifyZone, allows_classification
from cluster_class_voter import ClusterClassVoter
from lidar_object_tracker import LidarCluster, MapObject
from yolo_nav_fusion import (
  dedupe_same_class_detections,
  match_detection_to_cluster,
  parse_yolo_results,
)


def read_latest_frame(cap: cv2.VideoCapture) -> tuple[bool, Any]:
  """Drop buffered HTTP MJPEG frames and return the newest one."""
  if not cap.isOpened():
    return False, None
  for _ in range(5):
    if not cap.grab():
      return False, None
  return cap.retrieve()


def _open_stream_capture(stream_url: str) -> cv2.VideoCapture:
  cap = cv2.VideoCapture(stream_url)
  cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
  return cap


class MjpegStreamReader:
  """Robust MJPEG reader that parses multipart bytes directly.

  OpenCV VideoCapture can repeat the same buffered frame for HTTP MJPEG sources.
  This reader extracts JPEG frames by scanning for 0xFFD8..0xFFD9 markers.
  """

  def __init__(self, url: str):
    self.url = url
    self._lock = threading.Lock()
    self._latest_bgr = None
    self._last_frame_at: float = 0.0
    self._last_data_at: float = 0.0
    self._last_error: str = ""
    self._last_error_at: float = 0.0
    self._running = False

  def start(self) -> None:
    if self._running:
      return
    self._running = True
    threading.Thread(target=self._run, daemon=True).start()

  def stop(self) -> None:
    self._running = False

  def get_latest_bgr(self):
    with self._lock:
      return None if self._latest_bgr is None else self._latest_bgr.copy()

  def get_debug(self) -> dict[str, object]:
    with self._lock:
      age = (time.time() - self._last_frame_at) if self._last_frame_at else None
      data_age = (time.time() - self._last_data_at) if self._last_data_at else None
      return {
        "last_frame_at": self._last_frame_at,
        "last_frame_age_sec": age,
        "last_data_at": self._last_data_at,
        "last_data_age_sec": data_age,
        "last_error": self._last_error,
        "last_error_at": self._last_error_at,
      }

  def _run(self) -> None:
    buf = bytearray()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    while self._running:
      try:
        req = urllib.request.Request(
          self.url,
          headers={
            "User-Agent": "pinky_pro/robot_session",
            "Accept": "*/*",
            "Connection": "keep-alive",
          },
          method="GET",
        )
        with opener.open(req, timeout=3.0) as res:
          while self._running:
            # If the server keeps the connection open but stops sending, read() can stall.
            # Track last_data_at and force reconnect when no bytes arrive for a while.
            chunk = res.read(8192)
            if not chunk:
              break
            with self._lock:
              self._last_data_at = time.time()
            buf.extend(chunk)

            start = buf.find(b"\xff\xd8")
            end = buf.find(b"\xff\xd9", start + 2) if start != -1 else -1
            if start == -1 or end == -1:
              if len(buf) > 2_000_000:
                del buf[:-64_000]
              # Reconnect if we haven't received a decodable frame for too long.
              # This helps when intermediate network devices silently stall the stream.
              with self._lock:
                last_frame_at = self._last_frame_at
                last_data_at = self._last_data_at
              now = time.time()
              if last_data_at and (now - last_data_at) > 2.5:
                break
              if last_frame_at and (now - last_frame_at) > 5.0:
                break
              continue

            jpg = bytes(buf[start:end + 2])
            del buf[:end + 2]

            arr = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
              continue
            with self._lock:
              self._latest_bgr = frame
              self._last_frame_at = time.time()
      except Exception as exc:
        with self._lock:
          self._last_error = f"{type(exc).__name__}: {exc}"
          self._last_error_at = time.time()
        time.sleep(0.25)


@dataclass
class RobotSessionConfig:
  robot_id: str
  bridge_url: str
  stream_url: str

  # Fusion / matching parameters (reuse existing defaults)
  hfov_deg: float = 66.0
  match_angle_deg: float = 10.0
  cam_yaw_offset: float = 0.0
  label_confidence: float = 0.8
  classify_interval: float = 3.0
  classify_min_score: float = 2.0
  classify_dominance_ratio: float = 1.25
  # Empty list = allow class labeling everywhere on the map.
  classify_zones: list[ClassifyZone] | None = None

  # Polling intervals
  state_poll_hz: float = 10.0
  yolo_hz: float = 15.0


class BridgeTrackerView:
  """Read-only tracker view backed by the latest RobotBridge /api/state."""

  def __init__(self) -> None:
    self._objects: dict[str, MapObject] = {}

  def sync_from_state(self, state: dict[str, Any]) -> None:
    objects: dict[str, MapObject] = {}
    for raw in state.get("tracker_objects") or []:
      try:
        obj_id = str(raw["id"])
        objects[obj_id] = MapObject(
          id=obj_id,
          map_x=float(raw["map_x"]),
          map_y=float(raw["map_y"]),
          distance=float(raw.get("distance") or 0.0),
          bearing_deg=float(raw.get("bearing_deg") or 0.0),
          obj_class=raw.get("class"),
          confidence=float(raw.get("confidence") or 0.0),
          locked=bool(raw.get("locked")),
          last_seen=float(raw.get("last_seen") or time.time()),
        )
      except (KeyError, TypeError, ValueError):
        continue
    self._objects = objects

  def get_object_by_id(self, object_id: str) -> Optional[MapObject]:
    return self._objects.get(object_id)

  def find_object_near(
    self,
    map_x: float,
    map_y: float,
    tolerance: Optional[float] = None,
  ) -> Optional[MapObject]:
    tol = 0.2 if tolerance is None else tolerance
    best: Optional[MapObject] = None
    best_dist = tol
    for obj in self._objects.values():
      dist = math.hypot(obj.map_x - map_x, obj.map_y - map_y)
      if dist <= best_dist:
        best_dist = dist
        best = obj
    return best

  def find_canonical_vote_target(
    self,
    map_x: float,
    map_y: float,
    class_name: str,
  ) -> Optional[MapObject]:
    tol = 0.35
    cls = class_name.lower()
    candidates: list[MapObject] = []
    for obj in self._objects.values():
      if math.hypot(obj.map_x - map_x, obj.map_y - map_y) > tol:
        continue
      if cls and obj.locked and obj.obj_class != cls:
        continue
      candidates.append(obj)
    if not candidates:
      return None

    def sort_key(obj: MapObject) -> tuple[int, float]:
      locked_same = int(obj.locked and obj.obj_class == cls)
      dist = math.hypot(obj.map_x - map_x, obj.map_y - map_y)
      return (-locked_same, dist)

    return min(candidates, key=sort_key)


class RobotSession:
  def __init__(self, cfg: RobotSessionConfig, model: YOLO):
    self.cfg = cfg
    self.model = model
    self.model_names = {int(k): v for k, v in self.model.names.items()}
    self.class_voter = ClusterClassVoter(
      interval_sec=cfg.classify_interval,
      min_vote_score=cfg.classify_min_score,
      min_vote_confidence=cfg.label_confidence,
      dominance_ratio=cfg.classify_dominance_ratio,
    )
    self._tracker_view = BridgeTrackerView()

    self._lock = threading.Lock()
    self._latest_state: dict[str, Any] | None = None
    self._latest_frame_jpeg: bytes | None = None
    self._stream = MjpegStreamReader(cfg.stream_url)
    self._running = False

  def start(self) -> None:
    if self._running:
      return
    self._running = True
    self._stream.start()
    threading.Thread(target=self._state_poll_loop, daemon=True).start()
    threading.Thread(target=self._yolo_loop, daemon=True).start()

  def stop(self) -> None:
    self._running = False
    self._stream.stop()

  def get_latest_state(self) -> dict[str, Any] | None:
    with self._lock:
      if self._latest_state is None:
        return None
      return dict(self._latest_state)

  def get_latest_frame_jpeg(self) -> bytes | None:
    with self._lock:
      return self._latest_frame_jpeg

  def get_video_debug(self) -> dict[str, object]:
    return self._stream.get_debug()

  def _http_json(self, method: str, url: str, payload: Optional[dict[str, Any]] = None) -> Any:
    data = None
    headers = {}
    if payload is not None:
      data = json.dumps(payload).encode("utf-8")
      headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=1.5) as res:
      body = res.read().decode("utf-8")
      return json.loads(body) if body else None

  def _state_poll_loop(self) -> None:
    interval = 1.0 / max(self.cfg.state_poll_hz, 1.0)
    while self._running:
      try:
        state = self._http_json("GET", f"{self.cfg.bridge_url}/api/state")
        if isinstance(state, dict):
          with self._lock:
            self._latest_state = state
          self._tracker_view.sync_from_state(state)
      except (urllib.error.URLError, TimeoutError, ValueError):
        pass
      time.sleep(interval)

  def _clusters_from_state(self, state: dict[str, Any]) -> list[LidarCluster]:
    clusters: list[LidarCluster] = []
    for cl in (state.get("lidar_clusters") or []):
      try:
        clusters.append(
          LidarCluster(
            base_x=0.0,
            base_y=0.0,
            map_x=float(cl["map_x"]),
            map_y=float(cl["map_y"]),
            distance=float(cl.get("distance") or 0.0),
            bearing_rad=math.radians(float(cl.get("bearing_deg") or 0.0)),
            point_count=int(cl.get("point_count") or 0),
          )
        )
      except Exception:
        continue
    return clusters

  def _resolve_vote_target(
    self,
    cluster: LidarCluster,
    class_name: str,
  ) -> Optional[MapObject]:
    target = self._tracker_view.find_canonical_vote_target(
      cluster.map_x,
      cluster.map_y,
      class_name,
    )
    if target is None:
      target = self._tracker_view.find_object_near(cluster.map_x, cluster.map_y)
    return target

  def _push_label(self, map_x: float, map_y: float, cls: str, conf: float) -> None:
    try:
      self._http_json(
        "POST",
        f"{self.cfg.bridge_url}/api/labels/update",
        {"map_x": map_x, "map_y": map_y, "class": cls, "confidence": conf},
      )
    except (urllib.error.URLError, TimeoutError, ValueError):
      return

  def _publish_frame_jpeg(self, bgr_frame) -> None:
    ok, encoded = cv2.imencode(".jpg", bgr_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
    if ok:
      with self._lock:
        self._latest_frame_jpeg = encoded.tobytes()

  def _yolo_loop(self) -> None:
    interval = 1.0 / max(self.cfg.yolo_hz, 1.0)
    while self._running:
      frame = self._stream.get_latest_bgr()
      if frame is None:
        time.sleep(0.1)
        continue

      state = self.get_latest_state()
      has_pose = bool(state and state.get("pose"))

      h, w = frame.shape[:2]
      results = self.model(
        frame,
        conf=0.2,
        iou=0.6,
        imgsz=320,
        max_det=300,
        verbose=False,
      )

      if has_pose:
        pose = state["pose"]
        robot_yaw = float(pose.get("yaw") or 0.0)
        clusters = self._clusters_from_state(state)
        self._tracker_view.sync_from_state(state)

        detections = parse_yolo_results(results, self.model_names)
        detections = dedupe_same_class_detections(detections)

        used_cluster_keys: set[tuple[float, float]] = set()
        vote_winners: dict[tuple[str, tuple[float, float]], tuple[Any, LidarCluster, str]] = {}

        for det in detections:
          cluster = match_detection_to_cluster(
            det,
            clusters,
            robot_yaw,
            float(w),
            math.radians(self.cfg.hfov_deg),
            math.radians(self.cfg.match_angle_deg),
            self.cfg.cam_yaw_offset,
            tracker=None,
            class_voter=None,
            used_cluster_keys=used_cluster_keys,
          )
          if cluster is None or det.confidence < self.cfg.label_confidence:
            continue
          zones = self.cfg.classify_zones or []
          if not allows_classification(cluster.map_x, cluster.map_y, zones):
            continue

          cluster_key = (round(cluster.map_x, 3), round(cluster.map_y, 3))
          used_cluster_keys.add(cluster_key)
          target = self._resolve_vote_target(cluster, det.class_name)
          if target is None:
            continue

          vote_key = (det.class_name, cluster_key)
          prev = vote_winners.get(vote_key)
          if prev is None or det.confidence > prev[0].confidence:
            vote_winners[vote_key] = (det, cluster, target.id)

        for det, cluster, target_id in vote_winners.values():
          self.class_voter.record_match(target_id, det.class_name, det.confidence)

        for update in self.class_voter.compute_label_updates(self._tracker_view):
          self._push_label(
            float(update["map_x"]),
            float(update["map_y"]),
            str(update["class"]),
            float(update["confidence"]),
          )

      plotted = results[0].plot()
      self._publish_frame_jpeg(plotted)

      time.sleep(interval)
