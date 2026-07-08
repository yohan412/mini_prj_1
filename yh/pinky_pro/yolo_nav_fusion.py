#!/usr/bin/env python3
"""Match YOLO detections to LiDAR clusters and compute Nav2 goals."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from classify_zones import ClassifyZone, allows_classification
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


def _detection_xyxy(det: YoloDetection) -> tuple[float, float, float, float]:
  half_w = det.bbox_w / 2.0
  half_h = det.bbox_h / 2.0
  return (
    det.cx - half_w,
    det.cy - half_h,
    det.cx + half_w,
    det.cy + half_h,
  )


def _bbox_iou(a: YoloDetection, b: YoloDetection) -> float:
  ax1, ay1, ax2, ay2 = _detection_xyxy(a)
  bx1, by1, bx2, by2 = _detection_xyxy(b)
  ix1 = max(ax1, bx1)
  iy1 = max(ay1, by1)
  ix2 = min(ax2, bx2)
  iy2 = min(ay2, by2)
  if ix2 <= ix1 or iy2 <= iy1:
    return 0.0
  inter = (ix2 - ix1) * (iy2 - iy1)
  area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
  area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
  union = area_a + area_b - inter
  if union <= 0.0:
    return 0.0
  return inter / union


def dedupe_same_class_detections(
  detections: list[YoloDetection],
  *,
  iou_threshold: float = 0.25,
  center_dist_px: float = 48.0,
) -> list[YoloDetection]:
  """Drop lower-confidence boxes when same-class detections overlap in image space."""
  if len(detections) < 2:
    return detections

  ranked = sorted(detections, key=lambda det: det.confidence, reverse=True)
  kept: list[YoloDetection] = []
  for det in ranked:
    suppress = False
    for winner in kept:
      if winner.class_name != det.class_name:
        continue
      center_dist = math.hypot(det.cx - winner.cx, det.cy - winner.cy)
      if center_dist <= center_dist_px or _bbox_iou(det, winner) >= iou_threshold:
        suppress = True
        break
    if not suppress:
      kept.append(det)
  return kept


def match_detection_to_cluster(
  detection: YoloDetection,
  clusters: list[LidarCluster],
  robot_yaw: float,
  frame_width: float,
  hfov_rad: float,
  match_angle_rad: float,
  cam_yaw_offset: float = 0.0,
  tracker: Optional[LidarObjectTracker] = None,
  class_voter: Optional[ClusterClassVoter] = None,
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


def _apply_display_label(
  entry: dict[str, Any],
  tracker: LidarObjectTracker,
  class_voter: ClusterClassVoter,
  map_x: float,
  map_y: float,
) -> None:
  label = class_voter.get_label_near(tracker, map_x, map_y)
  if label is None:
    return
  entry['confirmed'] = True
  entry['labeled'] = label.labeled
  entry['class'] = label.class_name
  entry['confidence'] = label.confidence
  entry['vote_score'] = label.score
  entry['vote_scores'] = label.scores
  obj = tracker.get_object_by_id(label.object_id)
  if obj is not None:
    entry['map_x'] = obj.map_x
    entry['map_y'] = obj.map_y


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
  classify_zones: Optional[list[ClassifyZone]] = None,
) -> list[dict[str, Any]]:
  hfov_rad = math.radians(hfov_deg)
  match_angle_rad = math.radians(match_angle_deg)
  robot_yaw = robot_pose[2]
  fused: list[dict[str, Any]] = []
  used_cluster_keys: set[tuple[float, float]] = set()
  detections = dedupe_same_class_detections(detections)
  zones = classify_zones or []

  # Per cluster+class keep only the highest-confidence detection for voting.
  vote_winners: dict[tuple[str, tuple[float, float]], tuple[YoloDetection, LidarCluster, str]] = {}

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
      class_voter=class_voter,
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
      'bearing_deg': math.degrees(
        detection_bearing_robot(det, frame_width, robot_yaw, hfov_rad, cam_yaw_offset)
      ),
    }
    if cluster is None:
      fused.append(entry)
      continue

    cluster_key = (round(cluster.map_x, 3), round(cluster.map_y, 3))
    used_cluster_keys.add(cluster_key)
    entry['map_x'] = cluster.map_x
    entry['map_y'] = cluster.map_y
    entry['distance'] = cluster.distance
    in_zone = allows_classification(cluster.map_x, cluster.map_y, zones)
    entry['classify_allowed'] = in_zone

    if class_voter is not None:
      _apply_display_label(entry, tracker, class_voter, cluster.map_x, cluster.map_y)

      if in_zone and det.confidence >= label_confidence:
        target = tracker.find_canonical_vote_target(cluster.map_x, cluster.map_y, det.class_name)
        if target is None:
          target = tracker.find_object_near(cluster.map_x, cluster.map_y)
        if target is not None:
          vote_key = (det.class_name, cluster_key)
          prev = vote_winners.get(vote_key)
          if prev is None or det.confidence > prev[0].confidence:
            vote_winners[vote_key] = (det, cluster, target.id)

    fused.append(entry)

  if class_voter is not None:
    for det, _cluster, target_id in vote_winners.values():
      class_voter.record_match(target_id, det.class_name, det.confidence)
    class_voter.evaluate(tracker)
    for entry in fused:
      mx, my = entry.get('map_x'), entry.get('map_y')
      if mx is None or my is None:
        continue
      _apply_display_label(entry, tracker, class_voter, mx, my)

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


def resolve_fruit_object(
  tracker: LidarObjectTracker,
  fruit_class: str,
  robot_pose: Optional[tuple[float, float, float]],
  class_voter: Optional[ClusterClassVoter] = None,
) -> Optional[MapObject]:
  obj: Optional[MapObject] = None
  if class_voter is not None:
    obj = class_voter.find_object_by_class(tracker, fruit_class, robot_pose)
  if obj is None:
    obj = tracker.find_by_class(fruit_class, robot_pose)
  return obj


def resolve_fruit_target_xy(
  tracker: LidarObjectTracker,
  fruit_class: str,
  robot_pose: Optional[tuple[float, float, float]],
  class_voter: Optional[ClusterClassVoter] = None,
) -> Optional[tuple[float, float]]:
  obj = resolve_fruit_object(tracker, fruit_class, robot_pose, class_voter)
  if obj is None:
    return None
  return (obj.map_x, obj.map_y)


def resolve_fruit_goal(
  tracker: LidarObjectTracker,
  fruit_class: str,
  robot_pose: Optional[tuple[float, float, float]],
  approach_distance: float = 0.5,
  class_voter: Optional[ClusterClassVoter] = None,
) -> Optional[tuple[float, float, float]]:
  obj = resolve_fruit_object(tracker, fruit_class, robot_pose, class_voter)
  if obj is None or robot_pose is None:
    return None
  return compute_approach_goal(obj, robot_pose, approach_distance)
