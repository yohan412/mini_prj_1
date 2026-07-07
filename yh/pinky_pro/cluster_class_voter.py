#!/usr/bin/env python3
"""Accumulate per-cluster YOLO class votes and confirm labels every N seconds."""

from __future__ import annotations

import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
  from lidar_object_tracker import LidarObjectTracker


@dataclass
class VoteRecord:
  timestamp: float
  class_name: str
  score: float


@dataclass
class ClusterClassVoter:
  """Score YOLO-to-cluster matches over a sliding window, confirm winners periodically."""

  interval_sec: float = 3.0
  min_vote_score: float = 2.0
  min_vote_confidence: float = 0.8
  dominance_ratio: float = 1.25

  _votes: dict[str, list[VoteRecord]] = field(default_factory=dict)
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

  def evaluate(self, tracker: LidarObjectTracker) -> list[dict[str, object]]:
    """Run classification every interval_sec. Returns newly confirmed objects."""
    now = time.time()
    self._prune(now)
    if (now - self._last_eval) < self.interval_sec:
      return []
    self._last_eval = now

    confirmed: list[dict[str, object]] = []
    for obj_id in list(self._votes.keys()):
      obj = tracker.get_object_by_id(obj_id)
      if obj is None or obj.locked:
        continue

      scores = self.get_pending_scores(obj_id)
      if not scores:
        continue

      best_class, best_score = max(scores.items(), key=lambda item: item[1])
      if best_score < self.min_vote_score:
        continue

      other_score = sum(score for cls, score in scores.items() if cls != best_class)
      if other_score > 0.0 and best_score < other_score * self.dominance_ratio:
        continue

      vote_count = sum(
        1 for r in self._votes.get(obj_id, [])
        if r.class_name == best_class and r.timestamp >= now - self.interval_sec
      )
      avg_conf = best_score / max(vote_count, 1)
      if tracker.lock_object_class(obj_id, best_class, avg_conf):
        confirmed.append({
          'id': obj_id,
          'class': best_class,
          'score': best_score,
          'confidence': avg_conf,
        })
        self._votes.pop(obj_id, None)

    return confirmed
