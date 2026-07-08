#!/usr/bin/env python3
"""
Compatibility launcher for 4-class YOLO models.

Use this when your current best.pt only contains:
  apple, banana, orange, carrot

It reuses the existing yolo_nav_server.py implementation but overrides
TARGET_CLASS_NAMES to avoid failing on missing kitchen classes.
"""

from __future__ import annotations

import yolo_nav_server as server


server.TARGET_CLASS_NAMES = ("apple", "banana", "orange", "carrot")


def main() -> None:
  server.main()


if __name__ == "__main__":
  main()

