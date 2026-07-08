#!/usr/bin/env python3
"""LiDAR heading alignment and ultrasonic final approach for fruit goals."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class FinalApproachConfig:
  align_tolerance_rad: float = math.radians(8.0)
  align_angular_speed: float = 0.35
  approach_linear_speed: float = 0.06
  approach_angular_scale: float = 0.4
  ultrasonic_stop_distance: float = 0.10
  ultrasonic_min_valid: float = 0.03
  ultrasonic_max_valid: float = 2.5
  align_timeout: float = 25.0
  approach_timeout: float = 20.0
  # Legacy camera fields (kept for compatibility with old callers)
  align_tolerance_px: float = 20.0


def _normalize_angle(angle: float) -> float:
  while angle > math.pi:
    angle -= 2.0 * math.pi
  while angle < -math.pi:
    angle += 2.0 * math.pi
  return angle


def is_valid_ultrasonic(
  range_m: Optional[float],
  config: FinalApproachConfig,
) -> bool:
  if range_m is None or not math.isfinite(range_m):
    return False
  return config.ultrasonic_min_valid <= range_m <= config.ultrasonic_max_valid


def bearing_error_rad(
  robot_pose: tuple[float, float, float],
  target_xy: tuple[float, float],
) -> float:
  """Signed bearing error from robot yaw to target in map frame (rad)."""
  rx, ry, ryaw = robot_pose
  tx, ty = target_xy
  desired = math.atan2(ty - ry, tx - rx)
  return _normalize_angle(desired - ryaw)


def is_heading_aligned(
  robot_pose: tuple[float, float, float],
  target_xy: tuple[float, float],
  config: FinalApproachConfig,
) -> bool:
  return abs(bearing_error_rad(robot_pose, target_xy)) <= config.align_tolerance_rad


def compute_lidar_align_cmd(
  robot_pose: tuple[float, float, float],
  target_xy: tuple[float, float],
  config: FinalApproachConfig,
) -> tuple[float, float]:
  """Rotate in place until robot faces the LiDAR-labeled map target."""
  err = bearing_error_rad(robot_pose, target_xy)
  if abs(err) <= config.align_tolerance_rad:
    return 0.0, 0.0
  sign = 1.0 if err > 0.0 else -1.0
  return 0.0, sign * config.align_angular_speed


def compute_ultrasonic_approach_cmd(
  range_m: Optional[float],
  robot_pose: Optional[tuple[float, float, float]],
  target_xy: Optional[tuple[float, float]],
  config: FinalApproachConfig,
) -> tuple[float, float]:
  """Drive forward using ultrasonic distance; optional small LiDAR heading correction."""
  if not is_valid_ultrasonic(range_m, config):
    return 0.0, 0.0
  assert range_m is not None
  if range_m <= config.ultrasonic_stop_distance:
    return 0.0, 0.0

  # If we know the target bearing, prefer re-aligning first (rotate-in-place)
  # before moving forward. This keeps ultrasonic pointed at the target.
  if robot_pose is not None and target_xy is not None:
    err = bearing_error_rad(robot_pose, target_xy)
    if abs(err) > config.align_tolerance_rad:
      sign = 1.0 if err > 0.0 else -1.0
      return 0.0, sign * config.align_angular_speed

  return config.approach_linear_speed, 0.0


# ---- legacy camera helpers (unused by queue, kept for import compatibility) ----

def is_aligned(cx: float, frame_width: float, config: FinalApproachConfig) -> bool:
  offset = cx - (frame_width * 0.5)
  return abs(offset) <= config.align_tolerance_px


def compute_align_cmd(
  cx: float,
  frame_width: float,
  config: FinalApproachConfig,
) -> tuple[float, float]:
  offset = cx - (frame_width * 0.5)
  if abs(offset) <= config.align_tolerance_px:
    return 0.0, 0.0
  sign = 1.0 if offset > 0.0 else -1.0
  return 0.0, -sign * config.align_angular_speed


def compute_approach_cmd(
  range_m: Optional[float],
  cx: Optional[float],
  frame_width: Optional[float],
  config: FinalApproachConfig,
) -> tuple[float, float]:
  return compute_ultrasonic_approach_cmd(range_m, None, None, config)
