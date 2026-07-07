#!/usr/bin/env python3
"""
제어 PC에서 실행: 로봇이 송출하는 /camera/image_raw 토픽을 받아 YOLOv8 검출 후 창에 표시합니다.

필요 패키지: pip install ultralytics
"""

import cv2
import numpy as np
import rclpy
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = SCRIPT_DIR / 'yolov8n.pt'
WINDOW_NAME = 'Pinky YOLOv8 Detection (ROS)'
CONFIDENCE_THRESHOLD = 0.3
IOU_THRESHOLD = 0.6 # 겹치는 영역의 크기 비율 (0.0 ~ 1.0) 0.6 이상이면 하나의 객체로 인식
IMAGE_SIZE = 700 # 이미지 크기 800x800 픽셀 사이즈 너무 크면 검출 속도가 느려짐
MAX_DETECTIONS = 300 # 최대 검출 객체 수 300개 이상이면 검출 속도가 느려짐
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


def image_msg_to_bgr8(msg: Image) -> np.ndarray:
  """cv_bridge 없이 sensor_msgs/Image를 BGR numpy 배열로 변환합니다."""
  if msg.encoding == 'bgr8':
    channels = 3
    frame = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.step == msg.width * channels:
      return frame.reshape(msg.height, msg.width, channels)
    return frame.reshape(msg.height, msg.step)[:, : msg.width * channels].reshape(
      msg.height, msg.width, channels
    )

  if msg.encoding == 'rgb8':
    channels = 3
    frame = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.step == msg.width * channels:
      rgb = frame.reshape(msg.height, msg.width, channels)
    else:
      rgb = frame.reshape(msg.height, msg.step)[:, : msg.width * channels].reshape(
        msg.height, msg.width, channels
      )
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

  if msg.encoding == 'mono8':
    frame = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.step == msg.width:
      gray = frame.reshape(msg.height, msg.width)
    else:
      gray = frame.reshape(msg.height, msg.step)[:, : msg.width]
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

  raise ValueError(f'Unsupported image encoding: {msg.encoding}')


def is_window_closed(window_name: str) -> bool:
  try:
    return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
  except cv2.error:
    return True


class YoloRosViewer(Node):
  def __init__(self):
    super().__init__('yolo_ros_viewer')

    self.declare_parameter('topic_name', '/camera/image_raw')
    self.declare_parameter('model_path', str(DEFAULT_MODEL_PATH))
    self.declare_parameter('confidence', CONFIDENCE_THRESHOLD)
    self.declare_parameter('iou', IOU_THRESHOLD)
    self.declare_parameter('imgsz', IMAGE_SIZE)
    self.declare_parameter('max_det', MAX_DETECTIONS)

    topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
    model_path = Path(self.get_parameter('model_path').get_parameter_value().string_value)
    confidence = self.get_parameter('confidence').get_parameter_value().double_value
    iou = self.get_parameter('iou').get_parameter_value().double_value
    imgsz = self.get_parameter('imgsz').get_parameter_value().integer_value
    max_det = self.get_parameter('max_det').get_parameter_value().integer_value

    if not model_path.is_file():
      raise FileNotFoundError(f'YOLOv8 model not found: {model_path}')

    self.model = YOLO(str(model_path))
    self.target_class_ids = resolve_target_class_ids(self.model, TARGET_CLASS_NAMES)
    self.confidence = confidence
    self.iou = iou
    self.imgsz = imgsz
    self.max_det = max_det
    self.latest_frame = None
    self.should_exit = False

    self.create_subscription(Image, topic_name, self.image_callback, 10)
    self.create_timer(1.0 / 30.0, self.display_callback)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    self.get_logger().info(f'Subscribing to {topic_name}')
    self.get_logger().info(f'Using YOLOv8 model: {model_path}')
    self.get_logger().info(
      f'Detection limited to: {", ".join(TARGET_CLASS_NAMES)}'
    )
    self.get_logger().info(
      f'conf={self.confidence}, iou={self.iou}, imgsz={self.imgsz}, max_det={self.max_det}'
    )

  def image_callback(self, msg: Image):
    try:
      self.latest_frame = image_msg_to_bgr8(msg)
    except Exception as error:
      self.get_logger().warn(f'Failed to convert image: {error}')

  def display_callback(self):
    if self.should_exit:
      return

    if is_window_closed(WINDOW_NAME):
      self.should_exit = True
      rclpy.shutdown()
      return

    if self.latest_frame is None:
      return

    results = self.model(
      self.latest_frame,
      conf=self.confidence,
      iou=self.iou,
      imgsz=self.imgsz,
      max_det=self.max_det,
      classes=self.target_class_ids,
      verbose=False,
    )
    detected_frame = results[0].plot()
    cv2.imshow(WINDOW_NAME, detected_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
      self.should_exit = True
      rclpy.shutdown()


def main(args=None):
  rclpy.init(args=args)
  node = YoloRosViewer()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
  main()
