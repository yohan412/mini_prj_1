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
from ultralytics import YOLO

from classify_zones import ClassifyZone, allows_classification
from lidar_object_tracker import LidarCluster
from yolo_nav_fusion import (
  dedupe_same_class_detections,
  detection_bearing_robot,
  match_detection_to_cluster,
  parse_yolo_results,
)


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
  # Empty list = allow class labeling everywhere on the map.
  classify_zones: list[ClassifyZone] | None = None

  # Polling intervals
  state_poll_hz: float = 10.0
  yolo_hz: float = 15.0


class RobotSession:
  def __init__(self, cfg: RobotSessionConfig, model: YOLO):
    self.cfg = cfg
    self.model = model
    self.model_names = {int(k): v for k, v in self.model.names.items()}

    self._lock = threading.Lock()
    self._latest_state: dict[str, Any] | None = None
    self._latest_frame_jpeg: bytes | None = None
    self._running = False

  def start(self) -> None:
    if self._running:
      return
    self._running = True
    threading.Thread(target=self._state_poll_loop, daemon=True).start()
    threading.Thread(target=self._yolo_loop, daemon=True).start()

  def stop(self) -> None:
    self._running = False

  def get_latest_state(self) -> dict[str, Any] | None:
    with self._lock:
      if self._latest_state is None:
        return None
      return dict(self._latest_state)

  def get_latest_frame_jpeg(self) -> bytes | None:
    with self._lock:
      return self._latest_frame_jpeg

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

  def _push_label(self, map_x: float, map_y: float, cls: str, conf: float) -> None:
    try:
      self._http_json(
        "POST",
        f"{self.cfg.bridge_url}/api/labels/update",
        {"map_x": map_x, "map_y": map_y, "class": cls, "confidence": conf},
      )
    except (urllib.error.URLError, TimeoutError, ValueError):
      return

  def _yolo_loop(self) -> None:
    cap = cv2.VideoCapture(self.cfg.stream_url)
    if not cap.isOpened():
      return

    interval = 1.0 / max(self.cfg.yolo_hz, 1.0)
    while self._running:
      ok, frame = cap.read()
      if not ok or frame is None:
        time.sleep(0.1)
        continue

      state = self.get_latest_state()
      if not state or not state.get("pose"):
        time.sleep(interval)
        continue

      pose = state["pose"]
      robot_yaw = float(pose.get("yaw") or 0.0)
      clusters = self._clusters_from_state(state)

      h, w = frame.shape[:2]
      results = self.model(
        frame,
        conf=0.2,
        iou=0.6,
        imgsz=320,
        max_det=300,
        verbose=False,
      )
      detections = parse_yolo_results(results, self.model_names)
      detections = dedupe_same_class_detections(detections)

      used_cluster_keys: set[tuple[float, float]] = set()
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

        used_cluster_keys.add((round(cluster.map_x, 3), round(cluster.map_y, 3)))
        self._push_label(cluster.map_x, cluster.map_y, det.class_name, det.confidence)

      plotted = results[0].plot()
      ok2, encoded = cv2.imencode(".jpg", plotted, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
      if ok2:
        with self._lock:
          self._latest_frame_jpeg = encoded.tobytes()

      time.sleep(interval)

    cap.release()

