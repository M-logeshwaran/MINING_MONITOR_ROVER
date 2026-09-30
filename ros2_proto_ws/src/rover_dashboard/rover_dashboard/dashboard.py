#!/usr/bin/env python3
"""Web dashboard bridge for rover telemetry and thermal camera feeds."""

import base64
import math
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from flask import Flask, jsonify, send_from_directory
from flask_socketio import SocketIO
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage, Image, Joy
from std_msgs.msg import Bool, Float32, Int32, String

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


class DashboardNode(Node):
    """Publish ROS telemetry to the browser through Socket.IO."""

    def __init__(self):
        super().__init__("dashboard_node")
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("live_feed_topic", "/live_feed")
        self.declare_parameter("thermal_feed_topic", "/thermal_feed")
        self.declare_parameter("marked_feed_topic", "/live_marked_feed")
        self.declare_parameter("temperature_topic", "/drillpulse/temperature")
        self.declare_parameter("humidity_topic", "/drillpulse/humidity")
        self.declare_parameter("gas_reading_topic", "/drillpulse/gas")
        self.declare_parameter("gas_status_topic", "/drillpulse/gas_status")
        self.declare_parameter("left_distance_topic", "/drillpulse/left_distance")
        self.declare_parameter("right_distance_topic", "/drillpulse/right_distance")
        self.declare_parameter("speed_mode_topic", "/speed_mode")
        self.declare_parameter("motor_left_topic", "/drillpulse/motor_left")
        self.declare_parameter("motor_right_topic", "/drillpulse/motor_right")
        self.declare_parameter("status_topic", "/drillpulse/status")
        self.declare_parameter("joystick_topic", "/joystick_value")
        self.declare_parameter("battery_topic", "/battery")
        self.declare_parameter("lora_topic", "/lora")
        self.declare_parameter("wifihalow_topic", "/wifihalow")
        self.declare_parameter("rollback_status_topic", "/rollback_status")
        self.declare_parameter("autonomous_status_topic", "/autonomous_status")
        self.declare_parameter("full_control_topic", "/full_control")
        self.declare_parameter("odom_topic", "/virtual_odom")
        self.declare_parameter("path_topic", "/virtual_path")

        self.web_port = int(self.get_parameter("web_port").value)
        self.app = Flask(__name__, static_folder=WEB_DIR, static_url_path="")
        self.app.config["SECRET_KEY"] = "rover-dashboard"
        self.socketio = SocketIO(self.app, async_mode="threading", cors_allowed_origins="*")
        self._map_cache = None

        @self.app.route("/")
        def index():
            return send_from_directory(WEB_DIR, "index.html")

        @self.app.route("/api/map")
        def get_map():
            payload = self._get_map_payload()
            if payload is None:
                return jsonify({"error": "Map not found"}), 404
            return jsonify(payload)

        @self.socketio.on("connect")
        def on_connect():
            self.get_logger().info("Web dashboard client connected")

        self.bridge = CvBridge()
        self._image_executor = ThreadPoolExecutor(
            max_workers=2, thread_name_prefix="dashboard-image"
        )
        self._image_lock = threading.Lock()
        self._pending_images = {}
        self._image_workers = set()
        self._image_last_emit = {}
        self._image_min_interval = 1.0 / 15.0

        self._state_lock = threading.Lock()
        self._pending_state = {}
        self._state_event = threading.Event()
        self._state_running = True
        self._state_thread = threading.Thread(
            target=self._state_emit_loop,
            name="dashboard-state-emitter",
            daemon=True,
        )
        self._state_thread.start()

        image_qos = qos_profile_sensor_data
        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Image, self.get_parameter("live_feed_topic").value, self._on_live_feed, image_qos)
        self.create_subscription(CompressedImage, self.get_parameter("thermal_feed_topic").value, self._on_thermal_feed, image_qos)
        self.create_subscription(CompressedImage, self.get_parameter("marked_feed_topic").value, self._on_marked_feed, image_qos)
        self.create_subscription(Float32, self.get_parameter("temperature_topic").value, self._on_temperature, 10)
        self.create_subscription(Float32, self.get_parameter("humidity_topic").value, self._on_humidity, 10)
        self.create_subscription(Int32, self.get_parameter("gas_reading_topic").value, self._on_gas_reading, 10)
        self.create_subscription(Int32, self.get_parameter("gas_status_topic").value, self._on_gas_status, 10)
        self.create_subscription(Float32, self.get_parameter("left_distance_topic").value, self._on_left_distance, 10)
        self.create_subscription(Float32, self.get_parameter("right_distance_topic").value, self._on_right_distance, 10)
        self.create_subscription(Int32, self.get_parameter("speed_mode_topic").value, self._on_speed_mode, 10)
        self.create_subscription(Int32, self.get_parameter("motor_left_topic").value, self._on_motor_left, 10)
        self.create_subscription(Int32, self.get_parameter("motor_right_topic").value, self._on_motor_right, 10)
        self.create_subscription(String, self.get_parameter("status_topic").value, self._on_status, 10)
        self.create_subscription(Joy, self.get_parameter("joystick_topic").value, self._on_joystick, 10)
        self.create_subscription(Float32, self.get_parameter("battery_topic").value, self._on_battery, 10)
        self.create_subscription(Float32, self.get_parameter("lora_topic").value, self._on_lora, 10)
        self.create_subscription(Float32, self.get_parameter("wifihalow_topic").value, self._on_wifihalow, 10)
        self.create_subscription(String, self.get_parameter("rollback_status_topic").value, self._on_rollback_status, state_qos)
        self.create_subscription(String, self.get_parameter("autonomous_status_topic").value, self._on_autonomous_status, state_qos)
        self.create_subscription(Bool, self.get_parameter("full_control_topic").value, self._on_full_control, state_qos)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)
        self.create_subscription(Path, self.get_parameter("path_topic").value, self._on_path, 10)
        self.get_logger().info(f"Web dashboard available at http://0.0.0.0:{self.web_port}")

    def _get_map_payload(self):
        if self._map_cache is not None:
            return self._map_cache

        map_candidates = [
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "maps", "rover_map.pgm"),
            "/home/loki/maps/rover_map.pgm",
            "/maps/rover_map.pgm",
            os.path.expanduser("~/maps/rover_map.pgm"),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "maps", "rover_map.pgm"),
        ]
        yaml_candidates = [
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "maps", "rover_map.yaml"),
            "/home/loki/maps/rover_map.yaml",
            "/maps/rover_map.yaml",
            os.path.expanduser("~/maps/rover_map.yaml"),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "maps", "rover_map.yaml"),
        ]

        pgm_path = None
        for p in map_candidates:
            if os.path.exists(p):
                pgm_path = p
                break

        if not pgm_path:
            self.get_logger().warning("LiDAR map pgm not found in candidates")
            return None

        raw_img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
        if raw_img is None:
            self.get_logger().error(f"Failed to read LiDAR map image: {pgm_path}")
            return None

        h, w = raw_img.shape[:2]
        resolution = 0.05
        origin = [-25.558, -6.168, 0.0]

        for yp in yaml_candidates:
            if os.path.exists(yp):
                try:
                    with open(yp, "r") as f:
                        meta = yaml.safe_load(f)
                        if "resolution" in meta:
                            resolution = float(meta["resolution"])
                        if "origin" in meta:
                            origin = [float(v) for v in meta["origin"]]
                    break
                except Exception as exc:
                    self.get_logger().warning(f"Error parsing map yaml {yp}: {exc}")

        # Stylize map with mission-control aesthetic:
        # raw_img: 0 = obstacle (black), 205 = unknown (gray), 254 = free (white)
        rgba = np.zeros((h, w, 4), dtype=np.uint8)

        # Unknown space: very dark tinted mission blue
        rgba[raw_img == 205] = [8, 14, 22, 235]

        # Free space: tactical deep teal/slate floor
        rgba[raw_img == 254] = [18, 42, 58, 255]

        # Obstacles (0): high-contrast obstacle walls (danger red/coral)
        is_obs = (raw_img == 0).astype(np.uint8)
        rgba[is_obs > 0] = [255, 90, 90, 255]

        ok, png_bytes = cv2.imencode(".png", rgba)
        if not ok:
            return None
        png_b64 = base64.b64encode(png_bytes.tobytes()).decode("ascii")

        # Binary traversable grid (raw_img == 254: free space)
        traversable = (raw_img == 254).astype(np.uint8)

        # Extract largest connected component to ensure full reachability without disconnected doorways
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(traversable, connectivity=8)
        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        main_free = (labels == largest_label).astype(np.uint8)

        # Downsample cleanly by scale factor 2
        scale = 2
        small = cv2.resize(main_free, (w // scale, h // scale), interpolation=cv2.INTER_NEAREST)
        n_small, l_small, s_small, _ = cv2.connectedComponentsWithStats(small, connectivity=8)
        l_small_idx = 1 + np.argmax(s_small[1:, cv2.CC_STAT_AREA])
        main_small = (l_small == l_small_idx).astype(np.uint8)
        grid_b64 = base64.b64encode(main_small.tobytes()).decode("ascii")

        # Start and Goal points in the main connected corridor
        # Verified 171-step path avoiding all obstacles
        default_start_x, default_start_y = 305, 149
        default_goal_x, default_goal_y = 87, 407

        self._map_cache = {
            "image_data": f"data:image/png;base64,{png_b64}",
            "width": int(w),
            "height": int(h),
            "resolution": float(resolution),
            "origin": origin,
            "grid_scale": int(scale),
            "grid_width": int(w // scale),
            "grid_height": int(h // scale),
            "grid_b64": grid_b64,
            "default_start": [default_start_x, default_start_y],
            "default_goal": [default_goal_x, default_goal_y],
        }
        self.get_logger().info(f"Loaded and cached LiDAR map: {w}x{h} ({resolution}m/px), walkable cells: {int(np.sum(main_small))}")
        return self._map_cache

    def _on_live_feed(self, message):
        self._queue_image("live_feed", message)

    def _on_marked_feed(self, message):
        self._queue_image("live_marked_feed", message)

    def _on_thermal_feed(self, message):
        self._queue_image("thermal_feed", message)

    def _queue_image(self, event, message):
        """Keep only the newest frame so camera traffic cannot queue up."""
        with self._image_lock:
            self._pending_images[event] = message
            if event in self._image_workers:
                return
            self._image_workers.add(event)
        self._image_executor.submit(self._image_worker, event)

    def _image_worker(self, event):
        while True:
            with self._image_lock:
                message = self._pending_images.pop(event, None)
                if message is None:
                    self._image_workers.discard(event)
                    return
            self._emit_image(event, message)

    def _emit_image(self, event, message):
        now = time.monotonic()
        with self._image_lock:
            elapsed = now - self._image_last_emit.get(event, 0.0)
            if elapsed < self._image_min_interval:
                return
            self._image_last_emit[event] = now

        if isinstance(message, Image):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                ok, encoded = cv2.imencode(
                    ".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, 75]
                )
                if not ok:
                    return
                encoded_data = base64.b64encode(encoded.tobytes()).decode("ascii")
                self.socketio.emit(event, {"data_uri": f"data:image/jpeg;base64,{encoded_data}"})
                return
            except Exception as exc:
                self.get_logger().warning(f"Failed to convert raw image for {event}: {exc}")
                return

        image_format = (message.format or "jpeg").split(";")[0].split()[0].lower()
        if image_format == "jpg":
            image_format = "jpeg"
        if image_format not in ("jpeg", "png"):
            image_format = "jpeg"
        encoded = base64.b64encode(bytes(message.data)).decode("ascii")
        self.socketio.emit(event, {"data_uri": f"data:image/{image_format};base64,{encoded}"})

    def _on_temperature(self, message):
        self._queue_state("temperature", {"value": float(message.data)})

    def _on_humidity(self, message):
        self._queue_state("humidity", {"value": float(message.data)})

    def _on_gas_reading(self, message):
        self._queue_state("gas_reading", {"value": float(message.data)})

    def _on_gas_status(self, message):
        self._queue_state("gas_status", {"value": int(message.data)})

    def _on_left_distance(self, message):
        self._queue_state("left_distance", {"value": float(message.data)})

    def _on_right_distance(self, message):
        self._queue_state("right_distance", {"value": float(message.data)})

    def _on_speed_mode(self, message):
        self._queue_state("speed_mode", {"value": int(message.data)})

    def _on_motor_left(self, message):
        self._queue_state("motor_left", {"value": int(message.data)})

    def _on_motor_right(self, message):
        self._queue_state("motor_right", {"value": int(message.data)})

    def _on_status(self, message):
        self._queue_state("status", {"value": message.data})

    def _on_joystick(self, message):
        self._queue_state("joystick_values", {"axes": list(message.axes), "buttons": list(message.buttons)})

    def _on_battery(self, message):
        self._queue_state("battery", {"value": float(message.data)})

    def _on_lora(self, message):
        self._queue_state("lora", {"value": float(message.data)})

    def _on_wifihalow(self, message):
        self._queue_state("wifihalow", {"value": float(message.data)})

    def _on_rollback_status(self, message):
        self._queue_state("rollback_status", {"value": message.data})

    def _on_autonomous_status(self, message):
        self._queue_state("autonomous_status", {"value": message.data})

    def _on_full_control(self, message):
        self._queue_state("full_control", {"active": bool(message.data)})

    def _on_odom(self, message):
        orientation = message.pose.pose.orientation
        yaw = 2.0 * math.atan2(orientation.z, orientation.w)
        self._queue_state("odom", {
            "x": float(message.pose.pose.position.x),
            "y": float(message.pose.pose.position.y),
            "yaw": float(yaw),
            "linear_velocity": float(message.twist.twist.linear.x),
            "angular_velocity": float(message.twist.twist.angular.z),
        })

    def _on_path(self, message):
        self._queue_state("path", {
            "points": [
                {"x": float(pose.pose.position.x), "y": float(pose.pose.position.y)}
                for pose in message.poses
            ]
        })

    def _queue_state(self, event, payload):
        with self._state_lock:
            self._pending_state[event] = payload
        self._state_event.set()

    def _state_emit_loop(self):
        while self._state_running:
            self._state_event.wait(0.02)
            self._state_event.clear()
            with self._state_lock:
                pending = self._pending_state
                self._pending_state = {}
            for event, payload in pending.items():
                try:
                    self.socketio.emit(event, payload)
                except Exception as exc:
                    self.get_logger().warning(
                        f"Failed to emit dashboard event {event}: {exc}"
                    )

    def run_web_server(self):
        self.socketio.run(self.app, host="0.0.0.0", port=self.web_port, allow_unsafe_werkzeug=True)

    def close(self):
        self._state_running = False
        self._state_event.set()
        if self._state_thread.is_alive():
            self._state_thread.join(timeout=1.0)
        self._image_executor.shutdown(wait=False, cancel_futures=True)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    web_thread = threading.Thread(target=node.run_web_server, daemon=True)
    web_thread.start()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
