#!/usr/bin/env python3
"""Accumulate per-cluster YOLO class votes; label once, refresh class via voting."""

from __future__ import annotations

import math
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
  from lidar_object_tracker import LidarObjectTracker, MapObject


@dataclass
class VoteRecord:
  timestamp: float
  class_name: str
  score: float


@dataclass
class ObjectLabel:
  object_id: str
  class_name: str
  score: float
  confidence: float
  scores: dict[str, float]
  updated_at: float
  labeled: bool = True


@dataclass
class ClusterClassVoter:
  """Score YOLO-to-cluster matches; label clusters once, refresh every interval_sec."""

  interval_sec: float = 3.0
  min_vote_score: float = 2.0
  min_vote_confidence: float = 0.8
  dominance_ratio: float = 1.25

  _votes: dict[str, list[VoteRecord]] = field(default_factory=dict)
  _labels: dict[str, ObjectLabel] = field(default_factory=dict)
  _last_eval: float = field(default_factory=time.time)

  def record_match(
    self,
    object_id: str,
    class_name: str,
    confidence: float,
  ) -> None:
    """Record one scored vote. Low-confidence or unknown matches are ignored."""
    if confidence < self.min_vote_confidence:
      return
    cls = class_name.lower()
    if not cls:
      return
    self._votes.setdefault(object_id, []).append(
      VoteRecord(timestamp=time.time(), class_name=cls, score=confidence)
    )

  def _prune(self, now: float) -> None:
    cutoff = now - self.interval_sec
    for obj_id, records in list(self._votes.items()):
      kept = [r for r in records if r.timestamp >= cutoff]
      if kept:
        self._votes[obj_id] = kept
      else:
        del self._votes[obj_id]

  def get_pending_scores(self, object_id: str) -> dict[str, float]:
    now = time.time()
    cutoff = now - self.interval_sec
    scores: dict[str, float] = defaultdict(float)
    for record in self._votes.get(object_id, []):
      if record.timestamp >= cutoff:
        scores[record.class_name] += record.score
    return dict(scores)

  def _label_from_object(self, obj: MapObject, existing: Optional[ObjectLabel] = None) -> ObjectLabel:
    cls = (obj.obj_class or '').lower()
    return ObjectLabel(
      object_id=obj.id,
      class_name=cls,
      score=existing.score if existing else obj.confidence,
      confidence=obj.confidence,
      scores=existing.scores if existing else ({cls: obj.confidence} if cls else {}),
      updated_at=existing.updated_at if existing else time.time(),
      labeled=True,
    )

  def _prune_stale_labels(self, tracker: LidarObjectTracker) -> None:
    for obj_id in list(self._labels.keys()):
      if tracker.get_object_by_id(obj_id) is None:
        del self._labels[obj_id]

  def get_display_label(self, object_id: str) -> Optional[ObjectLabel]:
    return self._labels.get(object_id)

  def get_display_labels(self) -> list[ObjectLabel]:
    return list(self._labels.values())

  def is_labeled(self, object_id: str) -> bool:
    return object_id in self._labels

  def clear_all(self) -> int:
    """Drop all pending votes and display labels. Returns removed label count."""
    removed = len(self._labels)
    self._votes.clear()
    self._labels.clear()
    self._last_eval = 0.0
    return removed

  def get_label_near(
    self,
    tracker: LidarObjectTracker,
    map_x: float,
    map_y: float,
    class_name: str = '',
  ) -> Optional[ObjectLabel]:
    target = tracker.find_canonical_vote_target(map_x, map_y, class_name)
    if target is None:
      target = tracker.find_object_near(
        map_x,
        map_y,
        tolerance=tracker.same_class_merge_tolerance,
      )
    if target is None:
      return None
    label = self.get_display_label(target.id)
    if label is not None:
      return label
    if target.locked and target.obj_class:
      return self._label_from_object(target)
    return None

  def find_object_by_class(
    self,
    tracker: LidarObjectTracker,
    obj_class: str,
    robot_pose: Optional[tuple[float, float, float]] = None,
  ) -> Optional[MapObject]:
    self._prune_stale_labels(tracker)
    cls = obj_class.lower()
    candidates: list[tuple[MapObject, ObjectLabel]] = []
    for label in self._labels.values():
      if label.class_name != cls:
        continue
      obj = tracker.get_object_by_id(label.object_id)
      if obj is not None:
        candidates.append((obj, label))
    if not candidates:
      return tracker.find_by_class(obj_class, robot_pose)
    if robot_pose is None:
      return max(candidates, key=lambda item: item[1].score)[0]
    rx, ry, _ = robot_pose
    return min(
      candidates,
      key=lambda item: math.hypot(item[0].map_x - rx, item[0].map_y - ry),
    )[0]

  def _evaluate_scores(
    self,
    obj_id: str,
    now: float,
  ) -> Optional[tuple[str, float, float, dict[str, float]]]:
    records = [
      r for r in self._votes.get(obj_id, [])
      if r.timestamp >= now - self.interval_sec
    ]
    if not records:
      return None

    scores = self.get_pending_scores(obj_id)
    existing = self._labels.get(obj_id)

    if existing is not None:
      latest = max(records, key=lambda item: item.timestamp)
      if latest.confidence >= self.min_vote_confidence:
        best_class = latest.class_name
        best_score = scores.get(best_class, latest.confidence)
      else:
        best_class, best_score = max(scores.items(), key=lambda item: item[1])
    else:
      best_class, best_score = max(scores.items(), key=lambda item: item[1])
      if best_score < self.min_vote_score:
        return None

      other_score = sum(score for cls, score in scores.items() if cls != best_class)
      if other_score > 0.0 and best_score < other_score * self.dominance_ratio:
        return None

    vote_count = sum(
      1 for r in self._votes.get(obj_id, [])
      if r.class_name == best_class and r.timestamp >= now - self.interval_sec
    )
    avg_conf = best_score / max(vote_count, 1)
    return best_class, best_score, avg_conf, scores

  def compute_label_updates(
    self,
    tracker: LidarObjectTracker,
    *,
    now: Optional[float] = None,
  ) -> list[dict[str, object]]:
    """Return label updates without mutating tracker (for remote bridge sync)."""
    now = time.time() if now is None else now
    self._prune(now)
    self._prune_stale_labels(tracker)
    if (now - self._last_eval) < self.interval_sec:
      return []
    self._last_eval = now

    updated: list[dict[str, object]] = []
    candidate_ids = set(self._votes.keys()) | set(self._labels.keys())

    for obj_id in candidate_ids:
      obj = tracker.get_object_by_id(obj_id)
      if obj is None:
        continue

      result = self._evaluate_scores(obj_id, now)
      if result is None:
        continue

      best_class, best_score, avg_conf, scores = result
      prev = self._labels.get(obj_id)
      label = ObjectLabel(
        object_id=obj_id,
        class_name=best_class,
        score=best_score,
        confidence=avg_conf,
        scores=dict(scores),
        updated_at=now,
        labeled=True,
      )
      self._labels[obj_id] = label
      updated.append({
        'id': obj_id,
        'class': best_class,
        'score': best_score,
        'confidence': avg_conf,
        'scores': dict(scores),
        'map_x': obj.map_x,
        'map_y': obj.map_y,
        'labeled': True,
        'refreshed': prev is not None,
      })

    return updated

  def evaluate(self, tracker: LidarObjectTracker) -> list[dict[str, object]]:
    """Confirm or refresh labels every interval_sec from accumulated vote scores."""
    now = time.time()
    self._prune(now)
    self._prune_stale_labels(tracker)
    if (now - self._last_eval) < self.interval_sec:
      return []
    self._last_eval = now

    updated: list[dict[str, object]] = []
    candidate_ids = set(self._votes.keys()) | set(self._labels.keys())

    for obj_id in candidate_ids:
      obj = tracker.get_object_by_id(obj_id)
      if obj is None:
        continue

      result = self._evaluate_scores(obj_id, now)
      if result is None:
        continue

      best_class, best_score, avg_conf, scores = result
      if not tracker.update_object_class(obj_id, best_class, avg_conf):
        continue

      prev = self._labels.get(obj_id)
      label = ObjectLabel(
        object_id=obj_id,
        class_name=best_class,
        score=best_score,
        confidence=avg_conf,
        scores=dict(scores),
        updated_at=now,
        labeled=True,
      )
      self._labels[obj_id] = label
      updated.append({
        'id': obj_id,
        'class': best_class,
        'score': best_score,
        'confidence': avg_conf,
        'scores': dict(scores),
        'labeled': True,
        'refreshed': prev is not None,
      })

    return updated
