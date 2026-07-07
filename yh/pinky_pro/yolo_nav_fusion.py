#!/usr/bin/env python3
"""Match YOLO detections to LiDAR clusters and compute Nav2 goals."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from cluster_class_voter import ClusterClassVoter
from lidar_object_tracker import LidarCluster, LidarObjectTracker, MapObject


@dataclass
class YoloDetection:
  class_name: str
  confidence: float
  cx: float
  cy: float
  bbox_w: float
  bbox_h: float


def bbox_to_bearing_rad(cx: float, frame_width: float, hfov_rad: float) -> float:
  # Image x grows to the right; base_link/map bearing grows to the left (CCW).
  normalized = (cx / frame_width) - 0.5
  return -normalized * hfov_rad


def detection_bearing_robot(
  detection: YoloDetection,
  frame_width: float,
  robot_yaw: float,
  hfov_rad: float,
  cam_yaw_offset: float = 0.0,
) -> float:
  alpha_cam = bbox_to_bearing_rad(detection.cx, frame_width, hfov_rad)
  return robot_yaw + alpha_cam + cam_yaw_offset


def parse_yolo_results(results, model_names: dict[int, str]) -> list[YoloDetection]:
  detections: list[YoloDetection] = []
  if not results or results[0].boxes is None:
    return detections

  boxes = results[0].boxes
  for i in range(len(boxes)):
    cls_id = int(boxes.cls[i].item())
    conf = float(boxes.conf[i].item())
    xywh = boxes.xywh[i].tolist()
    class_name = model_names.get(cls_id, str(cls_id))
    detections.append(
      YoloDetection(
        class_name=class_name.lower(),
        confidence=conf,
        cx=xywh[0],
        cy=xywh[1],
        bbox_w=xywh[2],
        bbox_h=xywh[3],
      )
    )
  return detections


def match_detection_to_cluster(
  detection: YoloDetection,
  clusters: list[LidarCluster],
  robot_yaw: float,
  frame_width: float,
  hfov_rad: float,
  match_angle_rad: float,
  cam_yaw_offset: float = 0.0,
  tracker: Optional[LidarObjectTracker] = None,
  used_cluster_keys: Optional[set[tuple[float, float]]] = None,
) -> Optional[LidarCluster]:
  det_bearing = detection_bearing_robot(
    detection, frame_width, robot_yaw, hfov_rad, cam_yaw_offset
  )
  best_cluster = None
  best_diff = match_angle_rad
  for cluster in clusters:
    cluster_key = (round(cluster.map_x, 3), round(cluster.map_y, 3))
    if used_cluster_keys is not None and cluster_key in used_cluster_keys:
      continue

    if tracker is not None:
      nearby = tracker.find_object_near(cluster.map_x, cluster.map_y)
      if nearby is not None and nearby.locked and nearby.obj_class != detection.class_name:
        continue

    cluster_bearing_map = robot_yaw + cluster.bearing_rad
    diff = abs(_normalize_angle(det_bearing - cluster_bearing_map))
    if diff < best_diff:
      best_diff = diff
      best_cluster = cluster
  return best_cluster


def _normalize_angle(angle: float) -> float:
  while angle > math.pi:
    angle -= 2.0 * math.pi
  while angle < -math.pi:
    angle += 2.0 * math.pi
  return angle


def fuse_detections(
  tracker: LidarObjectTracker,
  detections: list[YoloDetection],
  clusters: list[LidarCluster],
  robot_pose: tuple[float, float, float],
  frame_width: float,
  *,
  hfov_deg: float = 66.0,
  match_angle_deg: float = 10.0,
  cam_yaw_offset: float = 0.0,
  label_confidence: float = 0.8,
  class_voter: Optional[ClusterClassVoter] = None,
) -> list[dict[str, Any]]:
  hfov_rad = math.radians(hfov_deg)
  match_angle_rad = math.radians(match_angle_deg)
  robot_yaw = robot_pose[2]
  fused: list[dict[str, Any]] = []
  used_cluster_keys: set[tuple[float, float]] = set()

  for det in detections:
    cluster = match_detection_to_cluster(
      det,
      clusters,
      robot_yaw,
      frame_width,
      hfov_rad,
      match_angle_rad,
      cam_yaw_offset,
      tracker=tracker,
      used_cluster_keys=used_cluster_keys,
    )
    entry: dict[str, Any] = {
      'class': det.class_name,
      'confidence': det.confidence,
      'cx': det.cx,
      'cy': det.cy,
      'distance': None,
      'map_x': None,
      'map_y': None,
      'confirmed': False,
      'locked': False,
      'bearing_deg': math.degrees(
        detection_bearing_robot(det, frame_width, robot_yaw, hfov_rad, cam_yaw_offset)
      ),
    }
    if cluster is None:
      fused.append(entry)
      continue

    cluster_key = (round(cluster.map_x, 3), round(cluster.map_y, 3))
    used_cluster_keys.add(cluster_key)

    nearby = tracker.find_object_near(cluster.map_x, cluster.map_y)
    if nearby is not None and nearby.locked:
      entry['confirmed'] = True
      entry['locked'] = True
      entry['class'] = nearby.obj_class
      entry['confidence'] = nearby.confidence
      entry['map_x'] = nearby.map_x
      entry['map_y'] = nearby.map_y
      entry['distance'] = math.hypot(nearby.map_x - robot_pose[0], nearby.map_y - robot_pose[1])
    elif class_voter is not None and det.confidence >= label_confidence:
      target = nearby
      if target is not None and not target.locked:
        class_voter.record_match(target.id, det.class_name, det.confidence)
      entry['map_x'] = cluster.map_x
      entry['map_y'] = cluster.map_y
      entry['distance'] = cluster.distance

    fused.append(entry)

  if class_voter is not None:
    class_voter.evaluate(tracker)
    for entry in fused:
      if entry.get('locked'):
        continue
      mx, my = entry.get('map_x'), entry.get('map_y')
      if mx is None or my is None:
        continue
      locked = tracker.find_object_near(mx, my)
      if locked is not None and locked.locked:
        entry['confirmed'] = True
        entry['locked'] = True
        entry['class'] = locked.obj_class
        entry['confidence'] = locked.confidence
        entry['map_x'] = locked.map_x
        entry['map_y'] = locked.map_y

  return fused


def compute_approach_goal(
  obj: MapObject,
  robot_pose: tuple[float, float, float],
  approach_distance: float = 0.5,
) -> tuple[float, float, float]:
  rx, ry, _ = robot_pose
  angle = math.atan2(obj.map_y - ry, obj.map_x - rx)
  goal_x = obj.map_x - approach_distance * math.cos(angle)
  goal_y = obj.map_y - approach_distance * math.sin(angle)
  return goal_x, goal_y, angle


def resolve_fruit_goal(
  tracker: LidarObjectTracker,
  fruit_class: str,
  robot_pose: Optional[tuple[float, float, float]],
  approach_distance: float = 0.5,
) -> Optional[tuple[float, float, float]]:
  obj = tracker.find_by_class(fruit_class, robot_pose)
  if obj is None or robot_pose is None:
    return None
  return compute_approach_goal(obj, robot_pose, approach_distance)
