#!/usr/bin/env python3
"""
Robot-side HTTP bridge for Nav2 + LiDAR tracking + command queue.

This process runs on each robot. It exposes a small HTTP API used by the
Control PC orchestrator/UI to:
- read robot state (pose/map/path/costmaps/cluster registry/queue)
- enqueue navigation commands (fruit/home/pose)
- stop active + clear pending commands
- update object labels (from Control PC YOLO fusion)
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from flask import Flask, jsonify, request
from geometry_msgs.msg import Twist
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

from classify_zones import allows_classification, parse_classify_zones
from command_queue import CommandQueue
from lidar_object_tracker import LidarObjectTracker, quat_to_yaw
from yolo_nav_fusion import resolve_fruit_goal, resolve_fruit_target_xy


app = Flask(__name__)
ros_node: Optional["RobotBridge"] = None


class RobotBridge(Node):
  def __init__(self, args: argparse.Namespace):
    super().__init__("robot_bridge")
    self.args = args

    self.lock = threading.Lock()
    self._frame_lock = threading.Lock()

    self.map_msg: OccupancyGrid | None = None
    self.path_msg: NavPath | None = None
    self.local_costmap_msg: Costmap | None = None
    self.global_costmap_msg: Costmap | None = None
    self.latest_scan: LaserScan | None = None
    self.latest_ultrasonic: float | None = None
    self.tf_pose: tuple[float, float, float] | None = None
    self._is_navigating: bool = False

    self.tracker = LidarObjectTracker(
      cluster_angle_tol_deg=args.cluster_angle_tol,
      cluster_dist_tol=args.cluster_dist_tol,
      object_ttl=args.object_ttl,
      locked_object_ttl=args.locked_object_ttl,
      map_match_tolerance=args.map_match_tolerance,
      label_confidence=args.label_confidence,
      cluster_merge_distance=args.cluster_merge_distance,
      cluster_merge_bearing_deg=args.cluster_merge_bearing_deg,
      same_class_merge_tolerance=args.same_class_merge_tolerance,
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
      stalled_timeout=args.stalled_timeout,
      stalled_move_tolerance=args.stalled_move_tolerance,
      stalled_yaw_tolerance=args.stalled_yaw_tolerance,
    )
    self.classify_zones = parse_classify_zones(getattr(args, "classify_zones", None))
    self._wire_queue_callbacks()

    map_qos = QoSProfile(
      history=QoSHistoryPolicy.KEEP_LAST,
      depth=1,
      reliability=QoSReliabilityPolicy.RELIABLE,
      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    self.create_subscription(OccupancyGrid, "map", self._map_cb, map_qos)
    self.create_subscription(NavPath, "plan", self._path_cb, 10)
    self.create_subscription(Costmap, "local_costmap/costmap", self._local_costmap_cb, 10)
    self.create_subscription(Costmap, "local_costmap/costmap_raw", self._local_costmap_cb, 10)
    self.create_subscription(Costmap, "global_costmap/costmap", self._global_costmap_cb, 10)
    self.create_subscription(Costmap, "global_costmap/costmap_raw", self._global_costmap_cb, 10)
    self.create_subscription(LaserScan, "scan", self._scan_cb, 10)
    self.create_subscription(Range, "us_sensor/range", self._ultrasonic_cb, 10)

    status_qos = QoSProfile(
      history=QoSHistoryPolicy.KEEP_LAST,
      depth=1,
      reliability=QoSReliabilityPolicy.RELIABLE,
      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    self.create_subscription(
      GoalStatusArray,
      "navigate_to_pose/_action/status",
      self._nav_status_cb,
      status_qos,
    )

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
    self.cancel_client = self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")
    self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

    self.create_timer(0.1, self._update_pose_from_tf)
    self.create_timer(0.1, self._process_tracking)
    self.create_timer(0.1, lambda: self.command_queue.tick())

    self.get_logger().info("RobotBridge started.")

  def _wire_queue_callbacks(self) -> None:
    self.command_queue.resolve_fruit_goal = self._queue_resolve_fruit
    self.command_queue.resolve_fruit_target_xy = self._queue_resolve_fruit_target_xy
    self.command_queue.resolve_home_goal = self._queue_resolve_home
    self.command_queue.send_goal = self.send_goal
    self.command_queue.cancel_navigation = self.cancel_goal
    self.command_queue.get_robot_pose = self.get_robot_pose
    self.command_queue.is_navigating = self.is_navigating
    self.command_queue.get_ultrasonic_range = self.get_ultrasonic_range
    self.command_queue.publish_cmd_vel = self.publish_cmd_vel
    self.command_queue.stop_robot = self.stop_robot

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
      if math.isfinite(msg.range):
        self.latest_ultrasonic = float(msg.range)
      else:
        self.latest_ultrasonic = None

  def get_ultrasonic_range(self) -> float | None:
    with self.lock:
      return self.latest_ultrasonic

  def _nav_status_cb(self, msg: GoalStatusArray) -> None:
    active = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
    navigating = any(s.status in active for s in msg.status_list)
    with self.lock:
      self._is_navigating = navigating

  def _update_pose_from_tf(self) -> None:
    try:
      trans = self.tf_buffer.lookup_transform("map", "base_link", Time())
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

  def _queue_resolve_home(self):
    if not self.args.home_pose:
      return None
    return (self.args.home_pose[0], self.args.home_pose[1], self.args.home_pose[2])

  def _queue_resolve_fruit(self, fruit_class: str):
    return resolve_fruit_goal(
      self.tracker,
      fruit_class.lower(),
      self.get_robot_pose(),
      self.args.approach_distance,
      class_voter=None,
    )

  def _queue_resolve_fruit_target_xy(self, fruit_class: str):
    return resolve_fruit_target_xy(
      self.tracker,
      fruit_class.lower(),
      self.get_robot_pose(),
      class_voter=None,
    )

  def _lookup_sensor_to_base(self, scan: LaserScan) -> tuple[float, float, float]:
    """TF: laser frame (e.g. rplidar_link) -> base_link."""
    frame_id = scan.header.frame_id if scan.header.frame_id else "rplidar_link"
    try:
      trans = self.tf_buffer.lookup_transform("base_link", frame_id, Time())
      t = trans.transform
      return (t.translation.x, t.translation.y, quat_to_yaw(t.rotation))
    except Exception as exc:
      self.get_logger().warn(
        f"TF base_link<-{frame_id} failed ({exc}); using rplidar 180deg fallback",
        throttle_duration_sec=5.0,
      )
      return (-0.017, 0.0, math.pi)

  def _process_tracking(self) -> None:
    with self.lock:
      scan = self.latest_scan
      pose = self.tf_pose
    if scan is None or pose is None:
      return
    sensor_to_base = self._lookup_sensor_to_base(scan)
    self.tracker.update(scan, pose, sensor_to_base=sensor_to_base)

  def publish_cmd_vel(self, linear: float, angular: float) -> None:
    msg = Twist()
    msg.linear.x = float(linear)
    msg.angular.z = float(angular)
    self.cmd_vel_pub.publish(msg)

  def stop_robot(self) -> None:
    self.publish_cmd_vel(0.0, 0.0)

  def send_goal(self, x: float, y: float, yaw: float) -> bool:
    if not self.nav_client.wait_for_server(timeout_sec=1.0):
      self.get_logger().error("navigate_to_pose not available")
      return False
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = self.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    self.get_logger().info(f"[NAV] goal x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
    self.nav_client.send_goal_async(goal)
    with self.lock:
      self._is_navigating = True
    return True

  def cancel_goal(self) -> bool:
    if not self.cancel_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().error("cancel_goal service not available")
      return False
    self.cancel_client.call_async(CancelGoal.Request())
    with self.lock:
      self._is_navigating = False
    return True

  def update_label_near(self, map_x: float, map_y: float, cls: str, confidence: float) -> bool:
    if not allows_classification(map_x, map_y, self.classify_zones):
      return False
    target = self.tracker.find_object_near(map_x, map_y, tolerance=self.tracker.same_class_merge_tolerance)
    if target is None:
      return False
    return self.tracker.update_object_class(target.id, cls.lower(), float(confidence))

  def clear_labels(self) -> int:
    return self.tracker.clear_all_classes()

  def get_state_snapshot(self) -> dict:
    with self.lock:
      map_msg = self.map_msg
      path_msg = self.path_msg
      local_costmap_msg = self.local_costmap_msg
      global_costmap_msg = self.global_costmap_msg
      tf_pose = self.tf_pose
      is_navigating = self._is_navigating

    def costmap_json(msg):
      if msg is None or len(msg.data) == 0:
        return None
      meta = msg.metadata
      return {
        "width": meta.size_x,
        "height": meta.size_y,
        "resolution": meta.resolution,
        "origin": {
          "x": meta.origin.position.x,
          "y": meta.origin.position.y,
          "yaw": quat_to_yaw(meta.origin.orientation),
        },
        "data": list(msg.data),
      }

    map_json = None
    if map_msg is not None:
      info = map_msg.info
      map_json = {
        "width": info.width,
        "height": info.height,
        "resolution": info.resolution,
        "origin": {
          "x": info.origin.position.x,
          "y": info.origin.position.y,
          "yaw": quat_to_yaw(info.origin.orientation),
        },
        "data": list(map_msg.data),
      }

    path_json = []
    if path_msg is not None:
      for ps in path_msg.poses:
        path_json.append({"x": ps.pose.position.x, "y": ps.pose.position.y})

    pose_json = None
    if tf_pose is not None:
      x, y, yaw = tf_pose
      pose_json = {"x": x, "y": y, "yaw": yaw}

    lidar_clusters = self.tracker.get_clusters_json()
    tracker_objects = [o.to_dict() for o in self.tracker.get_objects()]
    lidar_points = self.tracker.get_scan_points()
    return {
      "map": map_json,
      "pose": pose_json,
      "path": path_json,
      "local_costmap": costmap_json(local_costmap_msg),
      "global_costmap": costmap_json(global_costmap_msg),
      "navigating": is_navigating,
      "queue": self.command_queue.list_all(),
      "lidar_clusters": lidar_clusters,
      "lidar_points": lidar_points,
      "tracker_objects": tracker_objects,
      "ultrasonic_range": self.get_ultrasonic_range(),
    }


@app.get("/api/state")
def api_state():
  if ros_node is None:
    return jsonify({"error": "ROS not ready"}), 500
  return jsonify(ros_node.get_state_snapshot())


@app.post("/api/queue/add")
def api_queue_add():
  if ros_node is None:
    return jsonify({"success": False}), 500
  data = request.get_json() or {}
  cmd_type = data.get("type", "")
  params = data.get("params", {}) or {}

  # Compatibility: allow Control PC to send a computed pose for a "fruit" action.
  if cmd_type == "fruit" and all(k in params for k in ("x", "y")):
    cmd_type = "pose"

  cmd = ros_node.command_queue.add(cmd_type, params)
  return jsonify({"success": True, "command": cmd.to_dict()})


@app.post("/api/queue/clear")
def api_queue_clear():
  if ros_node is None:
    return jsonify({"success": False}), 500
  removed = ros_node.command_queue.clear_pending()
  return jsonify({"success": True, "removed": removed})


@app.post("/api/queue/stop_all")
def api_queue_stop_all():
  if ros_node is None:
    return jsonify({"success": False}), 500
  result = ros_node.command_queue.stop_all()
  return jsonify({"success": True, **result})


@app.post("/api/nav/stop")
def api_nav_stop():
  if ros_node is None:
    return jsonify({"success": False}), 500
  ros_node.command_queue.stop_active()
  return jsonify({"success": True})


@app.post("/api/labels/update")
def api_labels_update():
  if ros_node is None:
    return jsonify({"success": False}), 500
  data = request.get_json() or {}
  try:
    mx = float(data["map_x"])
    my = float(data["map_y"])
    cls = str(data["class"])
    conf = float(data.get("confidence", 0.0))
  except Exception:
    return jsonify({"success": False, "error": "invalid payload"}), 400

  ok = ros_node.update_label_near(mx, my, cls, conf)
  return jsonify({"success": ok})


@app.post("/api/labels/clear")
def api_labels_clear():
  if ros_node is None:
    return jsonify({"success": False}), 500
  cleared = ros_node.clear_labels()
  return jsonify({"success": True, "cleared": cleared})


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Robot-side Nav2 bridge")
  parser.add_argument("--host", default="0.0.0.0")
  parser.add_argument("--port", type=int, default=8091)

  # Queue/navigation parameters (reuse existing defaults)
  parser.add_argument("--approach-distance", type=float, default=0.3)
  parser.add_argument("--arrival-threshold", type=float, default=0.25)
  parser.add_argument("--detection-timeout", type=float, default=30.0)
  parser.add_argument("--goal-update-interval", type=float, default=2.0)
  parser.add_argument("--fine-approach-enabled", action=argparse.BooleanOptionalAction, default=True)
  parser.add_argument("--align-timeout", type=float, default=25.0)
  parser.add_argument("--approach-timeout", type=float, default=20.0)
  parser.add_argument("--align-tolerance-px", type=float, default=20.0)
  parser.add_argument("--align-angular-speed", type=float, default=0.4)
  parser.add_argument("--approach-linear-speed", type=float, default=0.06)
  parser.add_argument("--ultrasonic-stop-distance", type=float, default=0.10)

  # Stall detection for pose/home
  parser.add_argument("--stalled-timeout", type=float, default=5.0)
  parser.add_argument("--stalled-move-tolerance", type=float, default=0.03)
  parser.add_argument("--stalled-yaw-tolerance", type=float, default=0.05)

  # LiDAR tracker parameters
  parser.add_argument("--cluster-angle-tol", type=float, default=3.0)
  parser.add_argument("--cluster-dist-tol", type=float, default=0.15)
  parser.add_argument("--object-ttl", type=float, default=10.0)
  parser.add_argument("--locked-object-ttl", type=float, default=300.0)
  parser.add_argument("--map-match-tolerance", type=float, default=0.2)
  parser.add_argument("--label-confidence", type=float, default=0.8)
  parser.add_argument("--cluster-merge-distance", type=float, default=0.35)
  parser.add_argument("--cluster-merge-bearing-deg", type=float, default=8.0)
  parser.add_argument("--same-class-merge-tolerance", type=float, default=0.35)

  # Optional per-robot static home/exchange poses (can be overridden by config later)
  parser.add_argument("--home-pose", type=float, nargs=3, default=None, metavar=("X", "Y", "YAW"))
  parser.add_argument(
    "--classify-zones-file",
    default="",
    help="YAML/JSON with classify_zones rectangles; empty = allow labeling everywhere",
  )
  args = parser.parse_args()
  args.classify_zones = []
  if args.classify_zones_file:
    from pathlib import Path
    import json as _json

    zone_path = Path(args.classify_zones_file)
    if not zone_path.is_file():
      raise FileNotFoundError(f"Classify zones file not found: {zone_path}")
    text = zone_path.read_text(encoding="utf-8")
    if zone_path.suffix.lower() in {".yaml", ".yml"}:
      import yaml  # type: ignore

      loaded = yaml.safe_load(text) or {}
      raw = loaded.get("classify_zones", loaded) if isinstance(loaded, dict) else loaded
    else:
      loaded = _json.loads(text)
      raw = loaded.get("classify_zones", loaded) if isinstance(loaded, dict) else loaded
    args.classify_zones = parse_classify_zones(raw)
  return args


def main() -> None:
  global ros_node
  args = parse_args()
  rclpy.init()
  ros_node = RobotBridge(args)

  threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True).start()
  time.sleep(0.5)
  print(f"RobotBridge server: http://{args.host}:{args.port}")
  app.run(host=args.host, port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
  main()

