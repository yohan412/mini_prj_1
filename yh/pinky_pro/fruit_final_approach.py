#!/usr/bin/env python3
"""YOLO bearing alignment and ultrasonic final approach for fruit goals."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class FinalApproachConfig:
  align_tolerance_px: float = 20.0
  align_angular_speed: float = 0.4
  approach_linear_speed: float = 0.06
  approach_angular_scale: float = 0.5
  ultrasonic_stop_distance: float = 0.10
  ultrasonic_min_valid: float = 0.03
  ultrasonic_max_valid: float = 2.5
  align_timeout: float = 25.0
  approach_timeout: float = 20.0


def is_valid_ultrasonic(
  range_m: Optional[float],
  config: FinalApproachConfig,
) -> bool:
  if range_m is None or not math.isfinite(range_m):
    return False
  return config.ultrasonic_min_valid <= range_m <= config.ultrasonic_max_valid


def is_aligned(cx: float, frame_width: float, config: FinalApproachConfig) -> bool:
  offset = cx - (frame_width * 0.5)
  return abs(offset) <= config.align_tolerance_px


def compute_align_cmd(
  cx: float,
  frame_width: float,
  config: FinalApproachConfig,
) -> tuple[float, float]:
  """Rotate in place until the target is centered in the camera image."""
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
  """Drive forward using ultrasonic distance with optional YOLO centering."""
  if not is_valid_ultrasonic(range_m, config):
    return 0.0, 0.0
  assert range_m is not None
  if range_m <= config.ultrasonic_stop_distance:
    return 0.0, 0.0

  linear = config.approach_linear_speed
  angular = 0.0
  if cx is not None and frame_width:
    offset = cx - (frame_width * 0.5)
    if abs(offset) > config.align_tolerance_px:
      sign = 1.0 if offset > 0.0 else -1.0
      angular = -sign * config.align_angular_speed * config.approach_angular_scale
  return linear, angular
