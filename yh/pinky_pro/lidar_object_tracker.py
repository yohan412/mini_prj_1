#!/usr/bin/env python3
"""LiDAR scan clustering, static-map filtering, and map-frame object registry."""

from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Optional

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


@dataclass
class LidarCluster:
  base_x: float
  base_y: float
  map_x: float
  map_y: float
  distance: float
  bearing_rad: float
  point_count: int


@dataclass
class MapObject:
  id: str
  map_x: float
  map_y: float
  distance: float
  bearing_deg: float
  obj_class: Optional[str] = None
  confidence: float = 0.0
  locked: bool = False
  last_seen: float = field(default_factory=time.time)

  def to_dict(self) -> dict[str, Any]:
    return {
      'id': self.id,
      'class': self.obj_class,
      'map_x': self.map_x,
      'map_y': self.map_y,
      'distance': self.distance,
      'bearing_deg': self.bearing_deg,
      'confidence': self.confidence,
      'locked': self.locked,
      'last_seen': self.last_seen,
    }


def quat_to_yaw(q) -> float:
  siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
  cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny_cosp, cosy_cosp)


def lidar_point_to_base(
  lx: float,
  ly: float,
  tx: float,
  ty: float,
  yaw: float,
) -> tuple[float, float]:
  """Transform a 2D point from the laser sensor frame to base_link."""
  cos_y = math.cos(yaw)
  sin_y = math.sin(yaw)
  bx = tx + lx * cos_y - ly * sin_y
  by = ty + lx * sin_y + ly * cos_y
  return bx, by


def scan_to_base_points(
  scan: LaserScan,
  sensor_to_base: tuple[float, float, float],
) -> list[tuple[float, float, float]]:
  """Return list of (base_x, base_y, bearing_rad in base_link) for valid ranges."""
  tx, ty, yaw = sensor_to_base
  points: list[tuple[float, float, float]] = []
  angle = scan.angle_min
  for r in scan.ranges:
    if scan.range_min < r < scan.range_max and math.isfinite(r):
      lx = r * math.cos(angle)
      ly = r * math.sin(angle)
      bx, by = lidar_point_to_base(lx, ly, tx, ty, yaw)
      points.append((bx, by, math.atan2(by, bx)))
    angle += scan.angle_increment
  return points


def scan_to_points(scan: LaserScan) -> list[tuple[float, float, float]]:
  """Legacy helper: assumes scan frame equals base_link (no extrinsic)."""
  return scan_to_base_points(scan, (0.0, 0.0, 0.0))


def cluster_points(
  points: list[tuple[float, float, float]],
  *,
  angle_tol_rad: float = math.radians(3.0),
  dist_tol: float = 0.15,
) -> list[list[tuple[float, float, float]]]:
  if not points:
    return []

  clusters: list[list[tuple[float, float, float]]] = []
  current: list[tuple[float, float, float]] = [points[0]]

  for pt in points[1:]:
    prev = current[-1]
    angle_diff = abs(pt[2] - prev[2])
    dist_diff = abs(math.hypot(pt[0], pt[1]) - math.hypot(prev[0], prev[1]))
    if angle_diff <= angle_tol_rad and dist_diff <= dist_tol:
      current.append(pt)
    else:
      clusters.append(current)
      current = [pt]
  clusters.append(current)
  return clusters


def cluster_centroid(cluster: list[tuple[float, float, float]]) -> tuple[float, float, float]:
  xs = [p[0] for p in cluster]
  ys = [p[1] for p in cluster]
  cx = sum(xs) / len(xs)
  cy = sum(ys) / len(ys)
  return cx, cy, math.atan2(cy, cx)


def base_to_map(
  base_x: float,
  base_y: float,
  robot_x: float,
  robot_y: float,
  robot_yaw: float,
) -> tuple[float, float]:
  cos_y = math.cos(robot_yaw)
  sin_y = math.sin(robot_yaw)
  map_x = robot_x + base_x * cos_y - base_y * sin_y
  map_y = robot_y + base_x * sin_y + base_y * cos_y
  return map_x, map_y


def map_cell_value(map_msg: OccupancyGrid, map_x: float, map_y: float) -> Optional[int]:
  info = map_msg.info
  mx = int((map_x - info.origin.position.x) / info.resolution)
  my = int((map_y - info.origin.position.y) / info.resolution)
  if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
    return None
  index = my * info.width + mx
  if index < 0 or index >= len(map_msg.data):
    return None
  return int(map_msg.data[index])


def is_dynamic_cell(map_msg: OccupancyGrid, map_x: float, map_y: float) -> bool:
  """True when LiDAR hit lies in a free cell (not static wall)."""
  value = map_cell_value(map_msg, map_x, map_y)
  return value == 0


class LidarObjectTracker:
  def __init__(
    self,
    *,
    cluster_angle_tol_deg: float = 3.0,
    cluster_dist_tol: float = 0.15,
    object_ttl: float = 10.0,
    locked_object_ttl: float = 300.0,
    map_match_tolerance: float = 0.2,
    min_cluster_points: int = 2,
    label_confidence: float = 0.8,
  ):
    self.cluster_angle_tol_rad = math.radians(cluster_angle_tol_deg)
    self.cluster_dist_tol = cluster_dist_tol
    self.object_ttl = object_ttl
    self.locked_object_ttl = locked_object_ttl
    self.map_match_tolerance = map_match_tolerance
    self.min_cluster_points = min_cluster_points
    self.label_confidence = label_confidence

    self._lock = threading.Lock()
    self._map_msg: Optional[OccupancyGrid] = None
    self._objects: dict[str, MapObject] = {}
    self._latest_clusters: list[LidarCluster] = []
    self._latest_scan_points: list[dict[str, Any]] = []

  def set_map(self, map_msg: OccupancyGrid) -> None:
    with self._lock:
      self._map_msg = map_msg

  def update(
    self,
    scan: LaserScan,
    robot_pose: tuple[float, float, float],
    sensor_to_base: tuple[float, float, float] = (0.0, 0.0, 0.0),
  ) -> list[LidarCluster]:
    robot_x, robot_y, robot_yaw = robot_pose
    points = scan_to_base_points(scan, sensor_to_base)
    raw_clusters = cluster_points(
      points,
      angle_tol_rad=self.cluster_angle_tol_rad,
      dist_tol=self.cluster_dist_tol,
    )

    dynamic_clusters: list[LidarCluster] = []
    scan_points_map: list[dict[str, Any]] = []
    now = time.time()

    with self._lock:
      map_msg = self._map_msg

    for bx, by, bearing in points:
      mx, my = base_to_map(bx, by, robot_x, robot_y, robot_yaw)
      is_dynamic = map_msg is None or is_dynamic_cell(map_msg, mx, my)
      scan_points_map.append({
        'map_x': mx,
        'map_y': my,
        'dynamic': is_dynamic,
      })

    for cluster in raw_clusters:
      if len(cluster) < self.min_cluster_points:
        continue
      bx, by, bearing = cluster_centroid(cluster)
      mx, my = base_to_map(bx, by, robot_x, robot_y, robot_yaw)
      if map_msg is not None and not is_dynamic_cell(map_msg, mx, my):
        continue
      dist = math.hypot(bx, by)
      dynamic_clusters.append(
        LidarCluster(
          base_x=bx,
          base_y=by,
          map_x=mx,
          map_y=my,
          distance=dist,
          bearing_rad=bearing,
          point_count=len(cluster),
        )
      )

    with self._lock:
      self._latest_clusters = dynamic_clusters
      self._latest_scan_points = scan_points_map
      self._merge_clusters(dynamic_clusters, now)
      self._expire_objects(now)

    return dynamic_clusters

  def _merge_clusters(self, clusters: list[LidarCluster], now: float) -> None:
    for cluster in clusters:
      matched_id = None
      for obj_id, obj in self._objects.items():
        if math.hypot(obj.map_x - cluster.map_x, obj.map_y - cluster.map_y) <= self.map_match_tolerance:
          matched_id = obj_id
          break

      bearing_deg = math.degrees(cluster.bearing_rad)
      if matched_id is None:
        obj_id = f'obj_{uuid.uuid4().hex[:6]}'
        self._objects[obj_id] = MapObject(
          id=obj_id,
          map_x=cluster.map_x,
          map_y=cluster.map_y,
          distance=cluster.distance,
          bearing_deg=bearing_deg,
          last_seen=now,
        )
      else:
        obj = self._objects[matched_id]
        if not obj.locked:
          obj.map_x = cluster.map_x
          obj.map_y = cluster.map_y
          obj.distance = cluster.distance
          obj.bearing_deg = bearing_deg
        obj.last_seen = now

  def _expire_objects(self, now: float) -> None:
    expired: list[str] = []
    for oid, obj in self._objects.items():
      ttl = self.locked_object_ttl if obj.locked else self.object_ttl
      if (now - obj.last_seen) > ttl:
        expired.append(oid)
    for oid in expired:
      del self._objects[oid]

  def get_objects(self) -> list[MapObject]:
    with self._lock:
      return list(self._objects.values())

  def get_scan_points(self) -> list[dict[str, Any]]:
    with self._lock:
      return list(self._latest_scan_points)

  def get_locked_objects(self) -> list[MapObject]:
    with self._lock:
      return [o for o in self._objects.values() if o.locked]

  def find_object_near(
    self,
    map_x: float,
    map_y: float,
    tolerance: Optional[float] = None,
  ) -> Optional[MapObject]:
    tol = self.map_match_tolerance if tolerance is None else tolerance
    with self._lock:
      best: Optional[MapObject] = None
      best_dist = tol
      for obj in self._objects.values():
        dist = math.hypot(obj.map_x - map_x, obj.map_y - map_y)
        if dist <= best_dist:
          best_dist = dist
          best = obj
      return best

  def cluster_to_dict(self, cluster: LidarCluster, objects: Optional[dict[str, MapObject]] = None) -> dict[str, Any]:
    result: dict[str, Any] = {
      'map_x': cluster.map_x,
      'map_y': cluster.map_y,
      'distance': cluster.distance,
      'bearing_deg': math.degrees(cluster.bearing_rad),
      'point_count': cluster.point_count,
      'labeled': False,
      'class': None,
      'confidence': 0.0,
      'locked': False,
    }
    if objects is not None:
      for obj in objects.values():
        if not obj.locked or not obj.obj_class:
          continue
        if math.hypot(obj.map_x - cluster.map_x, obj.map_y - cluster.map_y) <= self.map_match_tolerance:
          result['labeled'] = True
          result['locked'] = True
          result['class'] = obj.obj_class
          result['confidence'] = obj.confidence
          result['map_x'] = obj.map_x
          result['map_y'] = obj.map_y
          break
    return result

  def get_clusters_json(self) -> list[dict[str, Any]]:
    with self._lock:
      return [self.cluster_to_dict(c, self._objects) for c in self._latest_clusters]

  def label_object(
    self,
    cluster_map_x: float,
    cluster_map_y: float,
    obj_class: str,
    confidence: float,
    min_confirm_confidence: Optional[float] = None,
  ) -> bool:
    threshold = self.label_confidence if min_confirm_confidence is None else min_confirm_confidence
    if confidence < threshold:
      return False

    with self._lock:
      best_id = None
      best_dist = self.map_match_tolerance
      for obj_id, obj in self._objects.items():
        dist = math.hypot(obj.map_x - cluster_map_x, obj.map_y - cluster_map_y)
        if dist < best_dist:
          best_dist = dist
          best_id = obj_id
      if best_id is None:
        return False

      obj = self._objects[best_id]
      if obj.locked:
        return False

      for other in self._objects.values():
        if other.id == obj.id or not other.locked or not other.obj_class:
          continue
        if other.obj_class == obj_class:
          continue
        if math.hypot(other.map_x - cluster_map_x, other.map_y - cluster_map_y) <= self.map_match_tolerance:
          return False

      obj.obj_class = obj_class
      obj.confidence = confidence
      obj.map_x = cluster_map_x
      obj.map_y = cluster_map_y
      obj.locked = True
      obj.last_seen = time.time()
      return True

  def lock_object_class(self, object_id: str, obj_class: str, confidence: float) -> bool:
    """Lock class from accumulated votes (no per-frame confidence gate)."""
    with self._lock:
      obj = self._objects.get(object_id)
      if obj is None or obj.locked:
        return False

      for other in self._objects.values():
        if other.id == obj.id or not other.locked or not other.obj_class:
          continue
        if other.obj_class == obj_class:
          continue
        if math.hypot(other.map_x - obj.map_x, other.map_y - obj.map_y) <= self.map_match_tolerance:
          return False

      obj.obj_class = obj_class
      obj.confidence = confidence
      obj.locked = True
      obj.last_seen = time.time()
      return True

  def get_object_by_id(self, object_id: str) -> Optional[MapObject]:
    with self._lock:
      return self._objects.get(object_id)

  def find_by_class(self, obj_class: str, robot_pose: Optional[tuple[float, float, float]]) -> Optional[MapObject]:
    with self._lock:
      candidates = [
        o for o in self._objects.values()
        if o.locked and o.obj_class == obj_class
      ]
    if not candidates:
      return None
    if robot_pose is None:
      return max(candidates, key=lambda o: o.confidence)
    rx, ry, _ = robot_pose
    return min(candidates, key=lambda o: math.hypot(o.map_x - rx, o.map_y - ry))
