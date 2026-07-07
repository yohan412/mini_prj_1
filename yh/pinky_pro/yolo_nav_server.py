#!/usr/bin/env python3
"""
PC-side Flask + ROS2 server: YOLO fruit navigation with command queue.

HTTP camera stream from robot, LiDAR object tracking on pre-mapped mini_prj_map.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from pathlib import Path

import cv2
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from flask import Flask, Response, jsonify, request, send_from_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Range
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO

from cluster_class_voter import ClusterClassVoter
from command_queue import CommandQueue
from lidar_object_tracker import LidarObjectTracker
from yolo_nav_fusion import fuse_detections, parse_yolo_results, resolve_fruit_goal

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = SCRIPT_DIR / 'best.pt'
DEFAULT_STREAM_URL = 'http://192.168.4.1:5000/video'
HOME_POSE_PATH = Path.home() / '.pinky_pro' / 'home_pose.json'
TARGET_CLASS_NAMES = ('apple', 'banana', 'orange', 'carrot')

app = Flask(__name__, static_folder=str(SCRIPT_DIR), static_url_path='')
ros_node = None


def quat_to_yaw(q) -> float:
  siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
  cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny_cosp, cosy_cosp)


def resolve_target_class_ids(model: YOLO, target_names: tuple[str, ...]) -> list[int]:
  model_names = {name.lower(): class_id for class_id, name in model.names.items()}
  class_ids = []
  for target_name in target_names:
    matched_id = model_names.get(target_name.lower())
    if matched_id is None:
      raise ValueError(f"Target class '{target_name}' not found in model.")
    if matched_id not in class_ids:
      class_ids.append(matched_id)
  return class_ids


def read_latest_frame(cap: cv2.VideoCapture):
  if not cap.isOpened():
    return False, None
  for _ in range(5):
    if not cap.grab():
      return False, None
  return cap.retrieve()


class YoloNavBridge(Node):
  def __init__(self, args: argparse.Namespace):
    super().__init__('yolo_nav_bridge')

    self.args = args
    self.lock = threading.Lock()

    self.map_msg = None
    self.path_msg = None
    self.local_costmap_msg = None
    self.global_costmap_msg = None
    self.latest_scan: LaserScan | None = None
    self.latest_ultrasonic: float | None = None
    self.tf_pose: tuple[float, float, float] | None = None
    self._is_navigating = False

    self.home_pose: dict | None = None
    self._load_home_pose()

    self.tracker = LidarObjectTracker(
      cluster_angle_tol_deg=args.cluster_angle_tol,
      cluster_dist_tol=args.cluster_dist_tol,
      object_ttl=args.object_ttl,
      locked_object_ttl=args.locked_object_ttl,
      map_match_tolerance=args.map_match_tolerance,
      label_confidence=args.label_confidence,
    )
    self.class_voter = ClusterClassVoter(
      interval_sec=args.classify_interval,
      min_vote_score=args.classify_min_score,
      min_vote_confidence=args.label_confidence,
      dominance_ratio=args.classify_dominance_ratio,
    )
    self.command_queue = CommandQueue(
      arrival_threshold=args.arrival_threshold,
      detection_timeout=args.detection_timeout,
      goal_update_interval=args.goal_update_interval,
      approach_distance=args.approach_distance,
      fine_approach_enabled=args.fine_approach_enabled,
      align_timeout=args.align_timeout,
      approach_timeout=args.approach_timeout,
      align_tolerance_px=args.align_tolerance_px,
      align_angular_speed=args.align_angular_speed,
      approach_linear_speed=args.approach_linear_speed,
      ultrasonic_stop_distance=args.ultrasonic_stop_distance,
    )
    self._wire_queue_callbacks()

    self.latest_detections: list[dict] = []
    self.latest_frame_jpeg: bytes | None = None
    self._frame_lock = threading.Lock()

    map_qos = QoSProfile(
      history=QoSHistoryPolicy.KEEP_LAST,
      depth=1,
      reliability=QoSReliabilityPolicy.RELIABLE,
      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    self.create_subscription(OccupancyGrid, 'map', self._map_cb, map_qos)
    self.create_subscription(NavPath, 'plan', self._path_cb, 10)
    self.create_subscription(Costmap, 'local_costmap/costmap', self._local_costmap_cb, 10)
    self.create_subscription(Costmap, 'local_costmap/costmap_raw', self._local_costmap_cb, 10)
    self.create_subscription(Costmap, 'global_costmap/costmap', self._global_costmap_cb, 10)
    self.create_subscription(Costmap, 'global_costmap/costmap_raw', self._global_costmap_cb, 10)
    self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
    self.create_subscription(Range, 'us_sensor/range', self._ultrasonic_cb, 10)

    status_qos = QoSProfile(
      history=QoSHistoryPolicy.KEEP_LAST,
      depth=1,
      reliability=QoSReliabilityPolicy.RELIABLE,
      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    self.create_subscription(
      GoalStatusArray,
      'navigate_to_pose/_action/status',
      self._nav_status_cb,
      status_qos,
    )

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
    self.create_timer(0.1, self._update_pose_from_tf)
    self.create_timer(0.1, self._process_perception)
    self.create_timer(0.1, lambda: self.command_queue.tick())

    self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    self.cancel_client = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
    self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    self.model = YOLO(str(args.model))
    self.target_class_ids = resolve_target_class_ids(self.model, TARGET_CLASS_NAMES)
    self.model_names = {int(k): v for k, v in self.model.names.items()}

    self.get_logger().info('YoloNavBridge started.')

  def _wire_queue_callbacks(self) -> None:
    self.command_queue.resolve_fruit_goal = self._queue_resolve_fruit
    self.command_queue.resolve_home_goal = self._queue_resolve_home
    self.command_queue.send_goal = self.send_goal
    self.command_queue.cancel_navigation = self.cancel_goal
    self.command_queue.get_robot_pose = self.get_robot_pose
    self.command_queue.is_navigating = self.is_navigating
    self.command_queue.get_fruit_detection = self._queue_get_fruit_detection
    self.command_queue.get_ultrasonic_range = self.get_ultrasonic_range
    self.command_queue.publish_cmd_vel = self.publish_cmd_vel
    self.command_queue.stop_robot = self.stop_robot

  def _load_home_pose(self) -> None:
    if HOME_POSE_PATH.is_file():
      try:
        data = json.loads(HOME_POSE_PATH.read_text(encoding='utf-8'))
        if all(k in data for k in ('x', 'y', 'yaw')):
          self.home_pose = {'x': data['x'], 'y': data['y'], 'yaw': data['yaw'], 'set': True}
      except (json.JSONDecodeError, OSError):
        pass

  def _save_home_pose(self, x: float, y: float, yaw: float) -> None:
    HOME_POSE_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {'x': x, 'y': y, 'yaw': yaw}
    HOME_POSE_PATH.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    self.home_pose = {**payload, 'set': True}

  def _map_cb(self, msg: OccupancyGrid) -> None:
    with self.lock:
      self.map_msg = msg
    self.tracker.set_map(msg)

  def _path_cb(self, msg: NavPath) -> None:
    with self.lock:
      self.path_msg = msg

  def _local_costmap_cb(self, msg: Costmap) -> None:
    with self.lock:
      self.local_costmap_msg = msg

  def _global_costmap_cb(self, msg: Costmap) -> None:
    with self.lock:
      self.global_costmap_msg = msg

  def _scan_cb(self, msg: LaserScan) -> None:
    with self.lock:
      self.latest_scan = msg

  def _ultrasonic_cb(self, msg: Range) -> None:
    with self.lock:
      if msg.range >= msg.min_range and msg.range <= msg.max_range and math.isfinite(msg.range):
        self.latest_ultrasonic = float(msg.range)
      else:
        self.latest_ultrasonic = None

  def get_ultrasonic_range(self) -> float | None:
    with self.lock:
      return self.latest_ultrasonic

  def publish_cmd_vel(self, linear: float, angular: float) -> None:
    msg = Twist()
    msg.linear.x = float(linear)
    msg.angular.z = float(angular)
    self.cmd_vel_pub.publish(msg)

  def stop_robot(self) -> None:
    self.publish_cmd_vel(0.0, 0.0)

  def _queue_get_fruit_detection(self, fruit_class: str) -> dict | None:
    fruit = fruit_class.lower()
    with self._frame_lock:
      frame = self._latest_bgr_frame
    if frame is None:
      return None
    _, w = frame.shape[:2]
    best = None
    best_conf = 0.0
    for det in self.latest_detections:
      if (det.get('class') or '').lower() != fruit:
        continue
      conf = float(det.get('confidence', 0.0))
      if conf < self.args.confidence:
        continue
      if conf > best_conf:
        best = det
        best_conf = conf
    if best is None:
      return None
    return {
      'cx': float(best['cx']),
      'cy': float(best['cy']),
      'confidence': best_conf,
      'frame_width': float(w),
    }

  def _nav_status_cb(self, msg: GoalStatusArray) -> None:
    active = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
    navigating = any(s.status in active for s in msg.status_list)
    with self.lock:
      self._is_navigating = navigating

  def _update_pose_from_tf(self) -> None:
    try:
      trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
      t = trans.transform
      with self.lock:
        self.tf_pose = (t.translation.x, t.translation.y, quat_to_yaw(t.rotation))
    except Exception:
      pass

  def get_robot_pose(self) -> tuple[float, float, float] | None:
    with self.lock:
      return self.tf_pose

  def is_navigating(self) -> bool:
    with self.lock:
      return self._is_navigating

  def _queue_resolve_fruit(self, fruit_class: str):
    return resolve_fruit_goal(
      self.tracker,
      fruit_class.lower(),
      self.get_robot_pose(),
      self.args.approach_distance,
    )

  def _queue_resolve_home(self):
    if not self.home_pose or not self.home_pose.get('set'):
      return None
    return (self.home_pose['x'], self.home_pose['y'], self.home_pose['yaw'])

  def _lookup_sensor_to_base(self, scan: LaserScan) -> tuple[float, float, float]:
    """TF: laser frame (e.g. rplidar_link) -> base_link."""
    frame_id = scan.header.frame_id if scan.header.frame_id else 'rplidar_link'
    try:
      trans = self.tf_buffer.lookup_transform('base_link', frame_id, Time())
      t = trans.transform
      return (t.translation.x, t.translation.y, quat_to_yaw(t.rotation))
    except Exception as exc:
      self.get_logger().warn(
        f'TF base_link<-{frame_id} failed ({exc}); using rplidar 180deg fallback',
        throttle_duration_sec=5.0,
      )
      return (-0.017, 0.0, math.pi)

  def _process_perception(self) -> None:
    with self.lock:
      scan = self.latest_scan
      pose = self.tf_pose
    if scan is None or pose is None:
      return
    sensor_to_base = self._lookup_sensor_to_base(scan)
    clusters = self.tracker.update(scan, pose, sensor_to_base)

    with self._frame_lock:
      frame = None
      if hasattr(self, '_latest_bgr_frame') and self._latest_bgr_frame is not None:
        frame = self._latest_bgr_frame.copy()

    if frame is None:
      return

    h, w = frame.shape[:2]
    results = self.model(
      frame,
      conf=self.args.confidence,
      iou=self.args.iou,
      imgsz=self.args.imgsz,
      max_det=self.args.max_det,
      classes=self.target_class_ids,
      verbose=False,
    )
    detections = parse_yolo_results(results, self.model_names)
    self.latest_detections = fuse_detections(
      self.tracker,
      detections,
      clusters,
      pose,
      float(w),
      hfov_deg=self.args.hfov_deg,
      match_angle_deg=self.args.object_match_angle,
      label_confidence=self.args.label_confidence,
      class_voter=self.class_voter,
    )

    plotted = results[0].plot()
    for det in self.latest_detections:
      if not det.get('confirmed'):
        continue
      label = det.get('class') or '?'
      if det.get('distance') is not None:
        label += f" {det['distance']:.1f}m"
      if det.get('locked'):
        label += ' [locked]'
        cv2.putText(
          plotted,
          label,
          (int(det['cx']), int(det['cy'])),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5,
          (0, 255, 255),
          1,
          cv2.LINE_AA,
        )

    ok, encoded = cv2.imencode('.jpg', plotted, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
    if ok:
      with self._frame_lock:
        self.latest_frame_jpeg = encoded.tobytes()

  def yolo_capture_loop(self) -> None:
    cap = cv2.VideoCapture(self.args.stream_url)
    if not cap.isOpened():
      self.get_logger().error(f'Failed to open stream: {self.args.stream_url}')
      return
    while rclpy.ok():
      ok, frame = read_latest_frame(cap)
      if ok and frame is not None:
        self._latest_bgr_frame = frame
      time.sleep(1.0 / 15.0)
    cap.release()

  def get_state_snapshot(self) -> dict:
    with self.lock:
      map_msg = self.map_msg
      path_msg = self.path_msg
      local_costmap_msg = self.local_costmap_msg
      global_costmap_msg = self.global_costmap_msg
      tf_pose = self.tf_pose
      is_navigating = self._is_navigating

    map_json = None
    if map_msg is not None:
      info = map_msg.info
      map_json = {
        'width': info.width,
        'height': info.height,
        'resolution': info.resolution,
        'origin': {
          'x': info.origin.position.x,
          'y': info.origin.position.y,
          'yaw': quat_to_yaw(info.origin.orientation),
        },
        'data': list(map_msg.data),
      }

    pose_json = None
    if tf_pose is not None:
      x, y, yaw = tf_pose
      pose_json = {'x': x, 'y': y, 'yaw': yaw}

    path_json = []
    if path_msg is not None:
      for ps in path_msg.poses:
        path_json.append({'x': ps.pose.position.x, 'y': ps.pose.position.y})

    def costmap_json(msg):
      if msg is None or len(msg.data) == 0:
        return None
      meta = msg.metadata
      return {
        'width': meta.size_x,
        'height': meta.size_y,
        'resolution': meta.resolution,
        'origin': {
          'x': meta.origin.position.x,
          'y': meta.origin.position.y,
          'yaw': quat_to_yaw(meta.origin.orientation),
        },
        'data': list(msg.data),
      }

    map_objects = [o.to_dict() for o in self.tracker.get_objects() if o.locked and o.obj_class]
    lidar_clusters = self.tracker.get_clusters_json()
    for cl in lidar_clusters:
      if cl.get('locked'):
        cl['status'] = 'confirmed'
        continue
      nearby = self.tracker.find_object_near(cl['map_x'], cl['map_y'])
      if nearby is None:
        cl['status'] = 'unknown'
        continue
      pending = self.class_voter.get_pending_scores(nearby.id)
      cl['status'] = 'pending' if pending else 'unknown'
      if pending:
        cl['pending_scores'] = pending
    lidar_points = self.tracker.get_scan_points()
    all_objects = [o.to_dict() for o in self.tracker.get_objects()]
    home = self.home_pose or {'x': 0, 'y': 0, 'yaw': 0, 'set': False}

    return {
      'map': map_json,
      'pose': pose_json,
      'path': path_json,
      'local_costmap': costmap_json(local_costmap_msg),
      'global_costmap': costmap_json(global_costmap_msg),
      'navigating': is_navigating,
      'queue': self.command_queue.list_all(),
      'home_pose': home,
      'detections': self.latest_detections,
      'map_objects': map_objects,
      'lidar_clusters': lidar_clusters,
      'lidar_points': lidar_points,
      'tracker_objects': all_objects,
      'ultrasonic_range': self.get_ultrasonic_range(),
    }

  def send_goal(self, x: float, y: float, yaw: float) -> bool:
    if not self.nav_client.wait_for_server(timeout_sec=1.0):
      self.get_logger().error('navigate_to_pose not available')
      return False
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = self.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    self.get_logger().info(f'[NAV] goal x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
    self.nav_client.send_goal_async(goal)
    with self.lock:
      self._is_navigating = True
    return True

  def set_initial_pose(self, x: float, y: float, yaw: float) -> bool:
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853891909122467
    self.initial_pose_pub.publish(msg)
    return True

  def cancel_goal(self) -> bool:
    if not self.cancel_client.wait_for_service(timeout_sec=1.0):
      return False
    self.cancel_client.call_async(CancelGoal.Request())
    with self.lock:
      self._is_navigating = False
    return True

  def get_mjpeg_frame(self) -> bytes | None:
    with self._frame_lock:
      return self.latest_frame_jpeg


def mjpeg_generator():
  boundary = b'--frame'
  while True:
    frame = ros_node.get_mjpeg_frame() if ros_node else None
    if frame is None:
      time.sleep(0.05)
      continue
    yield boundary + b'\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
    time.sleep(1.0 / 15.0)


@app.route('/')
def serve_index():
  return send_from_directory(str(SCRIPT_DIR), 'yolo_nav.html')


@app.route('/api/state')
def api_state():
  if ros_node is None:
    return jsonify({'error': 'ROS not ready'}), 500
  return jsonify(ros_node.get_state_snapshot())


@app.route('/api/video_feed')
def api_video_feed():
  return Response(mjpeg_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/queue/add', methods=['POST'])
def api_queue_add():
  if ros_node is None:
    return jsonify({'success': False}), 500
  data = request.get_json() or {}
  cmd_type = data.get('type', '')
  params = data.get('params', {})
  cmd = ros_node.command_queue.add(cmd_type, params)
  return jsonify({'success': True, 'command': cmd.to_dict()})


@app.route('/api/queue/remove', methods=['POST'])
def api_queue_remove():
  if ros_node is None:
    return jsonify({'success': False}), 500
  data = request.get_json() or {}
  ok = ros_node.command_queue.remove_pending(data.get('id', ''))
  return jsonify({'success': ok})


@app.route('/api/queue/clear', methods=['POST'])
def api_queue_clear():
  if ros_node is None:
    return jsonify({'success': False}), 500
  count = ros_node.command_queue.clear_pending()
  return jsonify({'success': True, 'removed': count})


@app.route('/api/home/set', methods=['POST'])
def api_home_set():
  if ros_node is None:
    return jsonify({'success': False}), 500
  pose = ros_node.get_robot_pose()
  if pose is None:
    return jsonify({'success': False, 'msg': 'Pose unavailable'}), 400
  ros_node._save_home_pose(pose[0], pose[1], pose[2])
  return jsonify({'success': True, 'home_pose': ros_node.home_pose})


@app.route('/api/home/go', methods=['POST'])
def api_home_go():
  if ros_node is None:
    return jsonify({'success': False}), 500
  cmd = ros_node.command_queue.add('home', {})
  return jsonify({'success': True, 'command': cmd.to_dict()})


@app.route('/api/nav/stop', methods=['POST'])
def api_nav_stop():
  if ros_node is None:
    return jsonify({'success': False}), 500
  ros_node.command_queue.stop_active()
  return jsonify({'success': True})


@app.route('/api/initialpose', methods=['POST'])
def api_initialpose():
  if ros_node is None:
    return jsonify({'success': False}), 500
  data = request.get_json() or {}
  ok = ros_node.set_initial_pose(float(data['x']), float(data['y']), float(data.get('yaw', 0.0)))
  return jsonify({'success': ok})


@app.route('/api/goal', methods=['POST'])
def api_goal():
  if ros_node is None:
    return jsonify({'success': False}), 500
  data = request.get_json() or {}
  cmd = ros_node.command_queue.add(
    'pose',
    {'x': float(data['x']), 'y': float(data['y']), 'yaw': float(data.get('yaw', 0.0))},
  )
  return jsonify({'success': True, 'command': cmd.to_dict()})


def ros_spin_thread():
  try:
    rclpy.spin(ros_node)
  finally:
    ros_node.destroy_node()
    rclpy.shutdown()


def parse_args():
  parser = argparse.ArgumentParser(description='Pinky YOLO navigation server')
  parser.add_argument('--stream-url', default=DEFAULT_STREAM_URL)
  parser.add_argument('--model', default=str(DEFAULT_MODEL_PATH))
  parser.add_argument('--port', type=int, default=8090)
  parser.add_argument('--host', default='0.0.0.0')
  parser.add_argument('--confidence', type=float, default=0.2)
  parser.add_argument('--iou', type=float, default=0.6)
  parser.add_argument('--imgsz', type=int, default=320)
  parser.add_argument('--max-det', type=int, default=300)
  parser.add_argument('--hfov-deg', type=float, default=66.0)
  parser.add_argument('--approach-distance', type=float, default=0.5)
  parser.add_argument('--arrival-threshold', type=float, default=0.3)
  parser.add_argument('--detection-timeout', type=float, default=30.0)
  parser.add_argument('--goal-update-interval', type=float, default=2.0)
  parser.add_argument('--cluster-angle-tol', type=float, default=3.0)
  parser.add_argument('--cluster-dist-tol', type=float, default=0.15)
  parser.add_argument('--object-match-angle', type=float, default=10.0)
  parser.add_argument('--object-ttl', type=float, default=10.0)
  parser.add_argument('--locked-object-ttl', type=float, default=300.0)
  parser.add_argument('--map-match-tolerance', type=float, default=0.2)
  parser.add_argument('--label-confidence', type=float, default=0.8,
                      help='Minimum YOLO confidence to count as a class vote')
  parser.add_argument('--classify-interval', type=float, default=3.0,
                      help='Seconds between cluster class scoring rounds')
  parser.add_argument('--classify-min-score', type=float, default=2.0,
                      help='Minimum accumulated vote score to confirm a class')
  parser.add_argument('--classify-dominance-ratio', type=float, default=1.25,
                      help='Winner score must exceed runner-up total by this ratio')
  parser.add_argument('--fine-approach-enabled', action=argparse.BooleanOptionalAction, default=True)
  parser.add_argument('--align-timeout', type=float, default=25.0)
  parser.add_argument('--approach-timeout', type=float, default=20.0)
  parser.add_argument('--align-tolerance-px', type=float, default=20.0)
  parser.add_argument('--align-angular-speed', type=float, default=0.4)
  parser.add_argument('--approach-linear-speed', type=float, default=0.06)
  parser.add_argument('--ultrasonic-stop-distance', type=float, default=0.10,
                      help='Stop forward approach when ultrasonic range falls below this (m)')
  return parser.parse_args()


def main():
  global ros_node
  args = parse_args()
  if not Path(args.model).is_file():
    raise FileNotFoundError(f'Model not found: {args.model}')

  rclpy.init()
  ros_node = YoloNavBridge(args)

  threading.Thread(target=ros_spin_thread, daemon=True).start()
  threading.Thread(target=ros_node.yolo_capture_loop, daemon=True).start()

  time.sleep(1.0)
  print(f'YoloNav server: http://{args.host}:{args.port}')
  app.run(host=args.host, port=args.port, debug=False, threaded=True)


if __name__ == '__main__':
  main()
