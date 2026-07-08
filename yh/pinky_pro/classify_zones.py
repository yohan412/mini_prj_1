#!/usr/bin/env python3
"""Map-frame rectangles that limit where cluster class votes may be applied."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Optional, Sequence


@dataclass(frozen=True)
class ClassifyZone:
  """Axis-aligned rectangle in the map frame (meters)."""

  min_x: float
  max_x: float
  min_y: float
  max_y: float
  name: str = ''

  def contains(self, map_x: float, map_y: float) -> bool:
    return self.min_x <= map_x <= self.max_x and self.min_y <= map_y <= self.max_y


def parse_classify_zones(raw: Optional[Iterable[Any]]) -> list[ClassifyZone]:
  """Parse zone dicts from CLI/YAML. Empty list means 'allow all map'."""
  zones: list[ClassifyZone] = []
  if not raw:
    return zones
  for item in raw:
    if not isinstance(item, dict):
      continue
    try:
      min_x = float(item['min_x'])
      max_x = float(item['max_x'])
      min_y = float(item['min_y'])
      max_y = float(item['max_y'])
    except (KeyError, TypeError, ValueError):
      continue
    if max_x < min_x:
      min_x, max_x = max_x, min_x
    if max_y < min_y:
      min_y, max_y = max_y, min_y
    zones.append(
      ClassifyZone(
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        name=str(item.get('name') or ''),
      )
    )
  return zones


def allows_classification(
  map_x: float,
  map_y: float,
  zones: Sequence[ClassifyZone],
) -> bool:
  """If zones is empty, classification is allowed everywhere."""
  if not zones:
    return True
  return any(zone.contains(map_x, map_y) for zone in zones)
