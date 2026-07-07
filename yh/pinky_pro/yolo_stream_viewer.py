#!/usr/bin/env python3
"""
제어 PC에서 실행: 로봇 HTTP MJPEG 스트림을 받아 YOLOv8 검출 후 창에 표시합니다.

필요 패키지: pip install ultralytics opencv-python
"""

import argparse
import time
import tkinter as tk
from datetime import datetime

import cv2
from pathlib import Path
from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = SCRIPT_DIR / 'yolo11n.pt'
DEFAULT_STREAM_URL = 'http://192.168.4.1:5000/video'
DEFAULT_SCREENSHOT_DIR = SCRIPT_DIR / 'screenshots'
AUTO_SCREENSHOT_INTERVAL_SEC = 5.0
WINDOW_NAME = 'Pinky YOLOv8 Stream Detection'
CONTROL_WINDOW_NAME = 'Pinky YOLO Controls'

CONFIDENCE_THRESHOLD = 0.2
IOU_THRESHOLD = 0.6
IMAGE_SIZE = 640
MAX_DETECTIONS = 300
TARGET_CLASS_NAMES = ('apple', 'banana', 'orange', 'carrot')


def resolve_target_class_ids(model: YOLO, target_names: tuple[str, ...]) -> list[int]:
  model_names = {name.lower(): class_id for class_id, name in model.names.items()}
  class_ids = []

  for target_name in target_names:
    matched_id = model_names.get(target_name.lower())
    if matched_id is None:
      available = ', '.join(model.names.values())
      raise ValueError(
        f"Target class '{target_name}' not found in model. Available: {available}"
      )

    if matched_id not in class_ids:
      class_ids.append(matched_id)

  return class_ids


def is_window_closed(window_name: str) -> bool:
  try:
    return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
  except cv2.error:
    return True


def read_latest_frame(cap: cv2.VideoCapture):
  if not cap.isOpened():
    return False, None

  for _ in range(5):
    if not cap.grab():
      return False, None

  return cap.retrieve()


def save_screenshot(frame, screenshot_dir: Path) -> Path:
  screenshot_dir.mkdir(parents=True, exist_ok=True)
  filename = screenshot_dir / f'capture_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}.jpg'
  if not cv2.imwrite(str(filename), frame):
    raise RuntimeError(f'Failed to save screenshot: {filename}')
  return filename


class ScreenshotControl:
  def __init__(self, on_screenshot, on_auto_start=None):
    self.on_screenshot = on_screenshot
    self.on_auto_start = on_auto_start
    self.closed = False
    self.auto_active = False

    self.root = tk.Tk()
    self.root.title(CONTROL_WINDOW_NAME)
    self.root.resizable(False, False)

    tk.Label(self.root, text='YOLO Detection Screenshot').pack(padx=12, pady=(12, 6))
    tk.Button(
      self.root,
      text='Screenshot',
      width=22,
      height=2,
      command=self.on_screenshot,
    ).pack(padx=12, pady=6)
    self.auto_button = tk.Button(
      self.root,
      text=f'Auto Screenshot ({int(AUTO_SCREENSHOT_INTERVAL_SEC)}s)',
      width=22,
      height=2,
      command=self._toggle_auto_screenshot,
    )
    self.auto_button.pack(padx=12, pady=6)
    tk.Label(self.root, text='Shortcut: s').pack(padx=12, pady=(0, 12))

    self.root.protocol('WM_DELETE_WINDOW', self.close)

  def _toggle_auto_screenshot(self):
    self.auto_active = not self.auto_active
    if self.auto_active:
      self.auto_button.config(text='Stop Auto Screenshot')
      if self.on_auto_start is not None:
        self.on_auto_start()
      print(f'Auto screenshot started (every {AUTO_SCREENSHOT_INTERVAL_SEC:.0f}s)')
    else:
      self.auto_button.config(text=f'Auto Screenshot ({int(AUTO_SCREENSHOT_INTERVAL_SEC)}s)')
      print('Auto screenshot stopped')

  def close(self):
    self.closed = True
    self.root.destroy()

  def update(self):
    if self.closed:
      return False
    self.root.update()
    return True


def parse_args():
  parser = argparse.ArgumentParser(description='Pinky HTTP stream YOLO viewer')
  parser.add_argument('--url', default=DEFAULT_STREAM_URL, help='MJPEG stream URL')
  parser.add_argument('--model', default=str(DEFAULT_MODEL_PATH), help='YOLOv8 model path')
  parser.add_argument('--confidence', type=float, default=CONFIDENCE_THRESHOLD)
  parser.add_argument('--iou', type=float, default=IOU_THRESHOLD)
  parser.add_argument('--imgsz', type=int, default=IMAGE_SIZE)
  parser.add_argument('--max-det', type=int, default=MAX_DETECTIONS)
  parser.add_argument(
    '--screenshot-dir',
    default=str(DEFAULT_SCREENSHOT_DIR),
    help='Directory to save screenshots',
  )
  parser.add_argument(
    '--yolo',
    action=argparse.BooleanOptionalAction,
    default=True,
    help='Enable YOLO detection (use --no-yolo to show raw stream only)',
  )
  return parser.parse_args()


def main():
  args = parse_args()
  model = None
  target_class_ids = None

  if args.yolo:
    model_path = Path(args.model)
    if not model_path.is_file():
      raise FileNotFoundError(f'YOLOv8 model not found: {model_path}')
    model = YOLO(str(model_path))
    target_class_ids = resolve_target_class_ids(model, TARGET_CLASS_NAMES)

  cap = cv2.VideoCapture(args.url)
  if not cap.isOpened():
    raise RuntimeError(f'Failed to open stream: {args.url}')

  cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
  print(f'Stream: {args.url}')
  print(f'YOLO: {"on" if args.yolo else "off"}')
  if args.yolo:
    print(f'Model: {args.model}')
    print(f'Classes: {", ".join(TARGET_CLASS_NAMES)}')
    print(f'conf={args.confidence}, iou={args.iou}, imgsz={args.imgsz}, max_det={args.max_det}')
  print(f'Screenshots: {args.screenshot_dir}')

  screenshot_dir = Path(args.screenshot_dir)
  latest_detected_frame = None
  screenshot_requested = False
  last_auto_screenshot_time = 0.0

  def request_screenshot():
    nonlocal screenshot_requested
    screenshot_requested = True

  def reset_auto_timer():
    nonlocal last_auto_screenshot_time
    last_auto_screenshot_time = time.monotonic()

  control = ScreenshotControl(request_screenshot, reset_auto_timer)

  try:
    while True:
      if is_window_closed(WINDOW_NAME) or not control.update():
        break

      ok, frame = read_latest_frame(cap)
      if not ok or frame is None:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
          break
        continue

      if args.yolo:
        results = model(
          frame,
          conf=args.confidence,
          iou=args.iou,
          imgsz=args.imgsz,
          max_det=args.max_det,
          classes=target_class_ids,
          verbose=False,
        )
        latest_detected_frame = results[0].plot()
      else:
        latest_detected_frame = frame

      cv2.imshow(WINDOW_NAME, latest_detected_frame)

      key = cv2.waitKey(1) & 0xFF
      if key == ord('q'):
        break
      if key == ord('s'):
        screenshot_requested = True

      if screenshot_requested and latest_detected_frame is not None:
        saved_path = save_screenshot(latest_detected_frame, screenshot_dir)
        print(f'Screenshot saved: {saved_path}')
        screenshot_requested = False

      if control.auto_active and latest_detected_frame is not None:
        now = time.monotonic()
        if now - last_auto_screenshot_time >= AUTO_SCREENSHOT_INTERVAL_SEC:
          saved_path = save_screenshot(latest_detected_frame, screenshot_dir)
          print(f'Auto screenshot saved: {saved_path}')
          last_auto_screenshot_time = now
  finally:
    control.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
  main()
