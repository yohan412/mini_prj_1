#!/usr/bin/env python3
"""
Recipe orchestrator for two robots (supplier/server) with both_arrived handoff.

This module is intentionally IO-light: it expects robot clients that can:
- enqueue commands
- fetch queue state
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional


class OrderStatus(str, Enum):
  IDLE = "idle"
  RUNNING = "running"
  COMPLETED = "completed"
  FAILED = "failed"
  CANCELLED = "cancelled"


@dataclass
class Recipe:
  name: str
  ingredient: str
  dish: str


@dataclass
class ExchangePose:
  x: float
  y: float
  yaw: float


class RobotClient:
  """Interface required by RecipeOrchestrator."""

  robot_id: str
  role: str  # supplier/server

  def queue_fruit(self, cls: str) -> bool:  # pragma: no cover
    raise NotImplementedError

  def queue_pose(self, x: float, y: float, yaw: float, label: str = "") -> bool:  # pragma: no cover
    raise NotImplementedError

  def stop_all(self) -> bool:  # pragma: no cover
    raise NotImplementedError

  def get_queue(self) -> list[dict[str, Any]]:  # pragma: no cover
    raise NotImplementedError


def _queue_has_running(queue: list[dict[str, Any]]) -> bool:
  return any(item.get("status") == "running" for item in queue)


def _queue_last_terminal(queue: list[dict[str, Any]]) -> Optional[dict[str, Any]]:
  # Queue list contains history + pending + active. We want the latest terminal record.
  for item in reversed(queue or []):
    if item.get("status") in ("completed", "failed", "cancelled"):
      return item
  return None


class RecipeOrchestrator:
  def __init__(
    self,
    *,
    supplier: RobotClient,
    server: RobotClient,
    exchange_pose_supplier: ExchangePose,
    exchange_pose_server: ExchangePose,
    poll_interval_sec: float = 0.25,
    detection_timeout: float = 30.0,
    handoff_timeout: float = 120.0,
  ):
    self.supplier = supplier
    self.server = server
    self.exchange_pose_supplier = exchange_pose_supplier
    self.exchange_pose_server = exchange_pose_server
    self.poll_interval_sec = poll_interval_sec
    self.detection_timeout = detection_timeout
    self.handoff_timeout = handoff_timeout

    self._lock = None  # reserved (threading added in server integration)
    self._status: OrderStatus = OrderStatus.IDLE
    self._message: str = ""
    self._active_order: Optional[str] = None
    self._active_step: str = ""
    self._started_at: float = 0.0

  def get_status(self) -> dict[str, Any]:
    return {
      "status": self._status.value,
      "order": self._active_order,
      "step": self._active_step,
      "message": self._message,
      "started_at": self._started_at,
    }

  def cancel(self) -> None:
    self._status = OrderStatus.CANCELLED
    self._message = "Cancelled by user"
    self._active_step = ""
    self._active_order = None
    self.supplier.stop_all()
    self.server.stop_all()

  def run_order(self, recipe: Recipe) -> dict[str, Any]:
    """Run one order synchronously (called from a worker thread)."""
    self._status = OrderStatus.RUNNING
    self._active_order = recipe.name
    self._started_at = time.time()

    try:
      # Step 1: supplier to ingredient
      self._active_step = "SupplierGotoIngredient"
      if not self.supplier.queue_fruit(recipe.ingredient):
        raise RuntimeError("Failed to enqueue supplier ingredient")
      self._wait_robot_terminal(self.supplier, timeout=self.detection_timeout)

      # Step 2: supplier to exchange
      self._active_step = "SupplierGotoExchange"
      if not self.supplier.queue_pose(
        self.exchange_pose_supplier.x,
        self.exchange_pose_supplier.y,
        self.exchange_pose_supplier.yaw,
        label="exchange",
      ):
        raise RuntimeError("Failed to enqueue supplier exchange")

      # Step 3: server to exchange (can be parallel; we enqueue immediately)
      self._active_step = "ServerGotoExchange"
      if not self.server.queue_pose(
        self.exchange_pose_server.x,
        self.exchange_pose_server.y,
        self.exchange_pose_server.yaw,
        label="exchange",
      ):
        raise RuntimeError("Failed to enqueue server exchange")

      # Step 4: wait both arrived at exchange
      self._active_step = "WaitHandoff"
      self._wait_both_exchange(timeout=self.handoff_timeout)

      # Step 5: server to dish
      self._active_step = "ServerGotoDish"
      if not self.server.queue_fruit(recipe.dish):
        raise RuntimeError("Failed to enqueue server dish")
      self._wait_robot_terminal(self.server, timeout=self.detection_timeout)

      # Step 6: server to customer/bell
      self._active_step = "ServerGotoCustomer"
      if not self.server.queue_fruit("bell"):
        raise RuntimeError("Failed to enqueue server customer")
      self._wait_robot_terminal(self.server, timeout=self.detection_timeout)

      self._status = OrderStatus.COMPLETED
      self._message = "Order completed"
      return self.get_status()
    except Exception as exc:
      self._status = OrderStatus.FAILED
      self._message = str(exc)
      self.supplier.stop_all()
      self.server.stop_all()
      return self.get_status()
    finally:
      self._active_step = ""
      self._active_order = None

  def _wait_robot_terminal(self, robot: RobotClient, timeout: float) -> None:
    started = time.time()
    while time.time() - started < timeout:
      q = robot.get_queue()
      term = _queue_last_terminal(q)
      if term is not None:
        status = term.get("status")
        if status == "completed":
          return
        if status in ("failed", "cancelled"):
          raise RuntimeError(f"{robot.robot_id} failed: {term.get('message','')}")
      time.sleep(self.poll_interval_sec)
    raise RuntimeError(f"{robot.robot_id} timeout")

  def _wait_both_exchange(self, timeout: float) -> None:
    started = time.time()
    supplier_done = False
    server_done = False
    while time.time() - started < timeout:
      sq = self.supplier.get_queue()
      bq = self.server.get_queue()
      supplier_done = supplier_done or self._exchange_completed(sq)
      server_done = server_done or self._exchange_completed(bq)
      if supplier_done and server_done:
        return
      time.sleep(self.poll_interval_sec)
    raise RuntimeError("handoff timeout (both_arrived)")

  def _exchange_completed(self, queue: list[dict[str, Any]]) -> bool:
    # If queue items include messages, we look for a terminal with message containing 'exchange'
    for item in reversed(queue or []):
      if item.get("status") != "completed":
        continue
      msg = str(item.get("message") or "")
      params = item.get("params") or {}
      if "exchange" in msg:
        return True
      if str(params.get("label") or "") == "exchange":
        return True
    return False

