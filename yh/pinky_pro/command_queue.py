#!/usr/bin/env python3
"""FIFO navigation command queue for fruit/home/pose goals."""

from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Optional


class CommandStatus(str, Enum):
  PENDING = 'pending'
  RUNNING = 'running'
  COMPLETED = 'completed'
  FAILED = 'failed'
  CANCELLED = 'cancelled'


class CommandType(str, Enum):
  FRUIT = 'fruit'
  HOME = 'home'
  POSE = 'pose'


@dataclass
class NavCommand:
  type: CommandType
  params: dict[str, Any]
  id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
  status: CommandStatus = CommandStatus.PENDING
  created_at: float = field(default_factory=time.time)
  started_at: Optional[float] = None
  finished_at: Optional[float] = None
  message: str = ''

  def to_dict(self) -> dict[str, Any]:
    return {
      'id': self.id,
      'type': self.type.value,
      'params': self.params,
      'status': self.status.value,
      'created_at': self.created_at,
      'started_at': self.started_at,
      'finished_at': self.finished_at,
      'message': self.message,
    }


GoalTuple = tuple[float, float, float]
DetectionDict = dict[str, float]
CmdVelTuple = tuple[float, float]


class CommandQueue:
  """FIFO queue with a single active command and worker tick loop."""

  def __init__(
    self,
    *,
    arrival_threshold: float = 0.3,
    detection_timeout: float = 30.0,
    goal_update_interval: float = 2.0,
    approach_distance: float = 0.5,
    fine_approach_enabled: bool = True,
    align_timeout: float = 10.0,
    approach_timeout: float = 20.0,
    align_tolerance_px: float = 20.0,
    align_angular_speed: float = 0.4,
    approach_linear_speed: float = 0.06,
    ultrasonic_stop_distance: float = 0.10,
    align_distance_tolerance: float = 0.05,
    stalled_timeout: float = 10.0,
    stalled_move_tolerance: float = 0.03,
    stalled_yaw_tolerance: float = 0.05,
  ):
    self.arrival_threshold = arrival_threshold
    self.detection_timeout = detection_timeout
    self.goal_update_interval = goal_update_interval
    self.approach_distance = approach_distance
    self.fine_approach_enabled = fine_approach_enabled
    self.align_timeout = align_timeout
    self.approach_timeout = approach_timeout
    self.align_tolerance_px = align_tolerance_px
    self.align_angular_speed = align_angular_speed
    self.approach_linear_speed = approach_linear_speed
    self.ultrasonic_stop_distance = ultrasonic_stop_distance
    self.align_distance_tolerance = align_distance_tolerance
    self.stalled_timeout = stalled_timeout
    self.stalled_move_tolerance = stalled_move_tolerance
    self.stalled_yaw_tolerance = stalled_yaw_tolerance

    self._lock = threading.Lock()
    self._commands: list[NavCommand] = []
    self._active: Optional[NavCommand] = None
    self._history: list[NavCommand] = []
    self._history_limit = 30
    self._last_goal_send = 0.0
    self._current_goal: Optional[GoalTuple] = None
    self._last_motion_pose: Optional[tuple[float, float, float]] = None
    self._last_motion_time: float = time.monotonic()

    self.resolve_fruit_goal: Optional[Callable[[str], Optional[GoalTuple]]] = None
    self.resolve_fruit_target_xy: Optional[
      Callable[[str], Optional[tuple[float, float]]]
    ] = None
    self.resolve_home_goal: Optional[Callable[[], Optional[GoalTuple]]] = None
    self.send_goal: Optional[Callable[[float, float, float], bool]] = None
    self.cancel_navigation: Optional[Callable[[], bool]] = None
    self.get_robot_pose: Optional[Callable[[], Optional[tuple[float, float, float]]]] = None
    self.is_navigating: Optional[Callable[[], bool]] = None
    self.get_fruit_detection: Optional[Callable[[str], Optional[DetectionDict]]] = None
    self.get_ultrasonic_range: Optional[Callable[[], Optional[float]]] = None
    self.publish_cmd_vel: Optional[Callable[[float, float], None]] = None
    self.stop_robot: Optional[Callable[[], None]] = None

  def add(self, cmd_type: str, params: Optional[dict[str, Any]] = None) -> NavCommand:
    command = NavCommand(
      type=CommandType(cmd_type),
      params=params or {},
    )
    with self._lock:
      self._commands.append(command)
    return command

  def list_all(self) -> list[dict[str, Any]]:
    with self._lock:
      items = [c.to_dict() for c in self._history]
      items.extend(c.to_dict() for c in self._commands)
      if self._active is not None:
        items.append(self._active.to_dict())
    return items

  def remove_pending(self, command_id: str) -> bool:
    with self._lock:
      for idx, cmd in enumerate(self._commands):
        if cmd.id == command_id and cmd.status == CommandStatus.PENDING:
          self._commands.pop(idx)
          return True
    return False

  def clear_pending(self) -> int:
    with self._lock:
      count = len(self._commands)
      self._commands.clear()
      return count

  def stop_active(self) -> None:
    if self.stop_robot:
      self.stop_robot()
    if self.cancel_navigation:
      self.cancel_navigation()
    with self._lock:
      if self._active is not None:
        self._active.status = CommandStatus.CANCELLED
        self._active.finished_at = time.time()
        self._active.message = 'Cancelled by user'
        self._history.append(self._active)
        if len(self._history) > self._history_limit:
          self._history = self._history[-self._history_limit:]
        self._active = None
      self._current_goal = None

  def stop_all(self) -> dict[str, int]:
    """Cancel the active command, clear pending queue, and stop the robot."""
    with self._lock:
      had_active = self._active is not None
    self.stop_active()
    removed_pending = self.clear_pending()
    return {
      'cancelled_active': int(had_active),
      'removed_pending': removed_pending,
    }

  def tick(self) -> None:
    with self._lock:
      if self._active is None:
        if not self._commands:
          return
        self._active = self._commands.pop(0)
        self._active.status = CommandStatus.RUNNING
        self._active.started_at = time.time()
        self._last_goal_send = 0.0
        self._current_goal = None
        self._reset_motion_tracker()

      active = self._active

    if active is None:
      return

    if active.type == CommandType.FRUIT:
      self._tick_fruit(active)
    elif active.type == CommandType.HOME:
      self._tick_home(active)
    elif active.type == CommandType.POSE:
      self._tick_pose(active)

  def _finish(self, command: NavCommand, status: CommandStatus, message: str = '') -> None:
    command.status = status
    command.message = message
    command.finished_at = time.time()
    with self._lock:
      if self._active is command:
        self._active = None
        self._current_goal = None
      self._history.append(command)
      if len(self._history) > self._history_limit:
        self._history = self._history[-self._history_limit:]

  def _maybe_send_goal(self, goal: GoalTuple, force: bool = False) -> bool:
    now = time.monotonic()
    if not force and (now - self._last_goal_send) < self.goal_update_interval:
      return False
    if self.send_goal is None:
      return False
    ok = self.send_goal(goal[0], goal[1], goal[2])
    if ok:
      self._last_goal_send = now
      self._current_goal = goal
    return ok

  def _arrived_at_goal(self, goal: GoalTuple) -> bool:
    if self.get_robot_pose is None:
      return False
    pose = self.get_robot_pose()
    if pose is None:
      return False
    rx, ry, _ = pose
    dist = math.hypot(goal[0] - rx, goal[1] - ry)
    navigating = self.is_navigating() if self.is_navigating else False
    return dist < self.arrival_threshold and not navigating

  def _reset_motion_tracker(self) -> None:
    self._last_motion_pose = None
    self._last_motion_time = time.monotonic()

  def _normalize_yaw_delta(self, a: float, b: float) -> float:
    delta = a - b
    while delta > math.pi:
      delta -= 2.0 * math.pi
    while delta < -math.pi:
      delta += 2.0 * math.pi
    return abs(delta)

  def _track_robot_motion(self) -> None:
    if self.get_robot_pose is None:
      return
    pose = self.get_robot_pose()
    if pose is None:
      return

    now = time.monotonic()
    if self._last_motion_pose is None:
      self._last_motion_pose = pose
      self._last_motion_time = now
      return

    rx, ry, ryaw = pose
    lx, ly, lyaw = self._last_motion_pose
    moved = (
      math.hypot(rx - lx, ry - ly) > self.stalled_move_tolerance
      or self._normalize_yaw_delta(ryaw, lyaw) > self.stalled_yaw_tolerance
    )
    if moved:
      self._last_motion_pose = pose
      self._last_motion_time = now

  def _stalled_duration(self) -> float:
    return time.monotonic() - self._last_motion_time

  def _fail_if_stalled(self, command: NavCommand, label: str) -> bool:
    """Return True when the command was failed due to stall timeout."""
    self._track_robot_motion()
    if self._stalled_duration() < self.stalled_timeout:
      return False
    if self.cancel_navigation:
      self.cancel_navigation()
    if self.stop_robot:
      self.stop_robot()
    self._finish(
      command,
      CommandStatus.FAILED,
      f'{label} stalled timeout ({self.stalled_timeout:.0f}s)',
    )
    return True

  def _align_nav_retries(self, command: NavCommand) -> int:
    try:
      return int(command.params.get('_align_nav_retries', 0))
    except (TypeError, ValueError):
      return 0

  def _retry_fruit_nav2_after_align_fail(
    self,
    command: NavCommand,
    fruit_class: str,
    reason: str,
  ) -> bool:
    """Restart Nav2 from the current pose after one align failure (single retry)."""
    if self._align_nav_retries(command) >= 1:
      return False

    if self.stop_robot:
      self.stop_robot()
    if self.cancel_navigation:
      self.cancel_navigation()

    command.params['_align_nav_retries'] = self._align_nav_retries(command) + 1
    for key in ('_target_x', '_target_y', '_phase_started'):
      command.params.pop(key, None)
    command.params['_phase'] = 'nav2'
    self._current_goal = None
    self._last_goal_send = 0.0
    self._reset_motion_tracker()
    command.message = f'Retry Nav2 to {fruit_class} ({reason})'
    return True

  def _handle_fruit_stall(
    self,
    command: NavCommand,
    fruit_class: str,
    label: str,
    target_xy: Optional[tuple[float, float]],
    target_distance: float,
    *,
    allow_ultrasonic_stop: bool = False,
    allow_nav_retry: bool = False,
  ) -> bool:
    """Return True when stalled and the command was finished (completed or failed)."""
    from fruit_final_approach import is_fruit_arrival_by_distance

    self._track_robot_motion()
    if self._stalled_duration() < self.stalled_timeout:
      return False

    if self.stop_robot:
      self.stop_robot()

    pose = self.get_robot_pose() if self.get_robot_pose else None
    ultrasonic = (
      self.get_ultrasonic_range() if self.get_ultrasonic_range else None
    )
    config = self._final_approach_config()
    arrived, detail = is_fruit_arrival_by_distance(
      pose,
      target_xy,
      ultrasonic,
      target_distance,
      config,
      allow_ultrasonic_stop=allow_ultrasonic_stop,
    )
    if arrived:
      self._finish(
        command,
        CommandStatus.COMPLETED,
        f'Arrived at {fruit_class} ({detail}, stall)',
      )
      return True

    if allow_nav_retry and self._retry_fruit_nav2_after_align_fail(
      command, fruit_class, 'stall',
    ):
      return True

    if self.cancel_navigation:
      self.cancel_navigation()
    self._finish(
      command,
      CommandStatus.FAILED,
      f'{label} stalled timeout ({self.stalled_timeout:.0f}s)',
    )
    return True

  def _tick_fruit(self, command: NavCommand) -> None:
    fruit_class = command.params.get('class', '')
    phase = command.params.get('_phase', 'nav2')

    if phase == 'nav2':
      self._tick_fruit_nav2(command, fruit_class)
    elif phase == 'align':
      self._tick_fruit_align(command, fruit_class)
    elif phase == 'approach':
      self._tick_fruit_approach(command, fruit_class)

  def _tick_fruit_nav2(self, command: NavCommand, fruit_class: str) -> None:
    if self.resolve_fruit_goal is None:
      return

    goal = self.resolve_fruit_goal(fruit_class)
    if goal is None:
      if command.started_at and (time.time() - command.started_at) > self.detection_timeout:
        self._finish(command, CommandStatus.FAILED, f'No {fruit_class} detected')
      return

    force = self._current_goal is None
    self._maybe_send_goal(goal, force=force)

    if self._current_goal and self._arrived_at_goal(self._current_goal):
      if self.fine_approach_enabled and self.publish_cmd_vel is not None:
        if self.cancel_navigation:
          self.cancel_navigation()
        if self.stop_robot:
          self.stop_robot()
        target_xy = self._fruit_target_xy(fruit_class)
        if target_xy is not None:
          command.params['_target_x'] = float(target_xy[0])
          command.params['_target_y'] = float(target_xy[1])
        command.params['_target_distance'] = float(self.approach_distance)
        self._reset_motion_tracker()
        command.params['_phase'] = 'align'
        command.params['_phase_started'] = time.time()
        command.message = f'LiDAR aligning to {fruit_class}'
        return
      self._finish(command, CommandStatus.COMPLETED, f'Arrived at {fruit_class}')

  def _final_approach_config(self):
    from fruit_final_approach import FinalApproachConfig

    return FinalApproachConfig(
      align_tolerance_px=self.align_tolerance_px,
      align_angular_speed=self.align_angular_speed,
      approach_linear_speed=self.approach_linear_speed,
      ultrasonic_stop_distance=self.ultrasonic_stop_distance,
      align_timeout=self.align_timeout,
      approach_timeout=self.approach_timeout,
      align_distance_tolerance=self.align_distance_tolerance,
    )

  def _fruit_target_xy(self, fruit_class: str) -> Optional[tuple[float, float]]:
    if self.resolve_fruit_target_xy is not None:
      return self.resolve_fruit_target_xy(fruit_class)
    return None

  def _frozen_fruit_target_xy(self, command: NavCommand, fruit_class: str) -> Optional[tuple[float, float]]:
    """Prefer frozen align-entry coordinates so heading does not chase a moving cluster."""
    if '_target_x' in command.params and '_target_y' in command.params:
      try:
        return (float(command.params['_target_x']), float(command.params['_target_y']))
      except (TypeError, ValueError):
        pass
    target_xy = self._fruit_target_xy(fruit_class)
    if target_xy is not None:
      command.params['_target_x'] = float(target_xy[0])
      command.params['_target_y'] = float(target_xy[1])
    return target_xy

  def _enter_approach(self, command: NavCommand, fruit_class: str, message: Optional[str] = None) -> None:
    if self.stop_robot:
      self.stop_robot()
    self._reset_motion_tracker()
    command.params['_phase'] = 'approach'
    command.params['_phase_started'] = time.time()
    command.message = message or f'Approaching {fruit_class}'

  def _frozen_target_distance(self, command: NavCommand) -> float:
    if '_target_distance' in command.params:
      try:
        return float(command.params['_target_distance'])
      except (TypeError, ValueError):
        pass
    return self.approach_distance

  def _tick_fruit_align(self, command: NavCommand, fruit_class: str) -> None:
    from fruit_final_approach import (
      compute_lidar_align_cmd,
      is_fine_align_complete,
      is_lidar_distance_at_target,
      is_valid_ultrasonic,
      lidar_range_to_target,
    )

    if self.publish_cmd_vel is None or self.get_robot_pose is None:
      self._finish(command, CommandStatus.FAILED, 'Fine approach not available')
      return

    pose = self.get_robot_pose()
    target_xy = self._frozen_fruit_target_xy(command, fruit_class)
    config = self._final_approach_config()
    target_distance = self._frozen_target_distance(command)
    ultrasonic_range = (
      self.get_ultrasonic_range() if self.get_ultrasonic_range else None
    )

    if pose is None or target_xy is None:
      command.message = f'Waiting for LiDAR target ({fruit_class})'
      if self.stop_robot:
        self.stop_robot()
      return

    lidar_range = lidar_range_to_target(pose, target_xy)
    us_text = (
      f'{ultrasonic_range:.2f}m'
      if is_valid_ultrasonic(ultrasonic_range, config)
      else 'n/a'
    )

    started = float(command.params.get('_phase_started', time.time()))
    if (time.time() - started) > self.align_timeout:
      if is_lidar_distance_at_target(
        lidar_range, target_distance, config.align_distance_tolerance,
      ):
        if self.stop_robot:
          self.stop_robot()
        self._finish(
          command,
          CommandStatus.COMPLETED,
          (
            f'Arrived at {fruit_class} '
            f'(LiDAR {lidar_range:.2f}m, target {target_distance:.2f}m)'
          ),
        )
      else:
        if self._retry_fruit_nav2_after_align_fail(
          command, fruit_class, 'align timeout',
        ):
          return
        self._enter_approach(
          command,
          fruit_class,
          f'Align timeout, approaching {fruit_class}',
        )
      return

    # Already on top of the cluster: heading is unreliable, go straight to ultrasonic.
    if lidar_range < 0.08:
      self._enter_approach(
        command,
        fruit_class,
        f'Target nearby, approaching {fruit_class}',
      )
      return

    if self._handle_fruit_stall(
      command,
      fruit_class,
      f'LiDAR align ({fruit_class})',
      target_xy,
      target_distance,
      allow_nav_retry=True,
    ):
      return

    if is_fine_align_complete(
      pose, target_xy, ultrasonic_range, target_distance, config,
    ):
      self._enter_approach(command, fruit_class)
      return

    linear, angular = compute_lidar_align_cmd(pose, target_xy, config)
    self.publish_cmd_vel(linear, angular)
    command.message = (
      f'LiDAR aligning to {fruit_class} '
      f'(d={lidar_range:.2f}m, us={us_text}, tgt={target_distance:.2f}m)'
    )

  def _tick_fruit_approach(self, command: NavCommand, fruit_class: str) -> None:
    from fruit_final_approach import (
      compute_ultrasonic_approach_cmd,
      compute_lidar_align_cmd,
      is_valid_ultrasonic,
    )

    started = float(command.params.get('_phase_started', time.time()))
    if (time.time() - started) > self.approach_timeout:
      if self.stop_robot:
        self.stop_robot()
      self._finish(command, CommandStatus.FAILED, f'Approach timeout for {fruit_class}')
      return

    if self.get_ultrasonic_range is None or self.publish_cmd_vel is None:
      self._finish(command, CommandStatus.FAILED, 'Ultrasonic approach not available')
      return

    config = self._final_approach_config()
    range_m = self.get_ultrasonic_range()
    pose = self.get_robot_pose() if self.get_robot_pose else None
    target_xy = self._frozen_fruit_target_xy(command, fruit_class)

    if not is_valid_ultrasonic(range_m, config):
      # Keep facing the target even before ultrasonic becomes valid.
      if pose is not None and target_xy is not None:
        linear, angular = compute_lidar_align_cmd(pose, target_xy, config)
        self.publish_cmd_vel(linear, angular)
        command.message = f'Waiting for ultrasonic, LiDAR aligning ({fruit_class})'
      else:
        command.message = f'Waiting for ultrasonic ({fruit_class})'
        if self.stop_robot:
          self.stop_robot()
      return

    assert range_m is not None
    if self._handle_fruit_stall(
      command,
      fruit_class,
      f'Approach ({fruit_class})',
      target_xy,
      self._frozen_target_distance(command),
      allow_ultrasonic_stop=True,
    ):
      return

    linear, angular = compute_ultrasonic_approach_cmd(
      range_m, pose, target_xy, config,
    )
    if linear == 0.0 and angular == 0.0:
      if self.stop_robot:
        self.stop_robot()
      self._finish(
        command,
        CommandStatus.COMPLETED,
        f'Arrived at {fruit_class} ({range_m:.2f}m)',
      )
      return

    self.publish_cmd_vel(linear, angular)
    command.message = f'Approaching {fruit_class} ({range_m:.2f}m)'

  def _tick_home(self, command: NavCommand) -> None:
    if self.resolve_home_goal is None:
      return
    goal = self.resolve_home_goal()
    if goal is None:
      self._finish(command, CommandStatus.FAILED, 'Home pose not set')
      return

    force = self._current_goal is None
    self._maybe_send_goal(goal, force=force)

    if self._current_goal and self._arrived_at_goal(self._current_goal):
      self._finish(command, CommandStatus.COMPLETED, 'Arrived at home')

  def _tick_pose(self, command: NavCommand) -> None:
    if self._fail_if_stalled(command, 'Pose navigation'):
      return

    try:
      x = float(command.params['x'])
      y = float(command.params['y'])
      yaw = float(command.params.get('yaw', 0.0))
    except (KeyError, TypeError, ValueError):
      self._finish(command, CommandStatus.FAILED, 'Invalid pose params')
      return

    goal = (x, y, yaw)
    force = self._current_goal is None
    self._maybe_send_goal(goal, force=force)

    if self._current_goal and self._arrived_at_goal(self._current_goal):
      self._finish(command, CommandStatus.COMPLETED, 'Arrived at pose')
