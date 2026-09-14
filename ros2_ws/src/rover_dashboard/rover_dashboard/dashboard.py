#!/usr/bin/env python3
"""Web dashboard bridge for rover telemetry and thermal camera feeds."""

import base64
import os
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, send_from_directory
from flask_socketio import SocketIO
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Joy
from std_msgs.msg import Float32, Int32, String

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
        self.declare_parameter("speed_mode_topic", "/drillpulse/speed_mode")
        self.declare_parameter("motor_left_topic", "/drillpulse/motor_left")
        self.declare_parameter("motor_right_topic", "/drillpulse/motor_right")
        self.declare_parameter("status_topic", "/drillpulse/status")
        self.declare_parameter("joystick_topic", "/joystick_values")

        self.web_port = int(self.get_parameter("web_port").value)
        self.app = Flask(__name__, static_folder=WEB_DIR, static_url_path="")
        self.app.config["SECRET_KEY"] = "rover-dashboard"
        self.socketio = SocketIO(self.app, async_mode="threading", cors_allowed_origins="*")

        @self.app.route("/")
        def index():
            return send_from_directory(WEB_DIR, "index.html")

        @self.socketio.on("connect")
        def on_connect():
            self.get_logger().info("Web dashboard client connected")

        self.bridge = CvBridge()

        image_qos = qos_profile_sensor_data
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
        self.get_logger().info(f"Web dashboard available at http://0.0.0.0:{self.web_port}")

    def _on_live_feed(self, message):
        self._emit_image("live_feed", message)

    def _on_marked_feed(self, message):
        self._emit_image("live_feed", message)
        self._emit_image("live_marked_feed", message)

    def _on_thermal_feed(self, message):
        self._emit_image("thermal_feed", message)

    def _emit_image(self, event, message):
        if isinstance(message, Image):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                ok, encoded = cv2.imencode(".jpg", cv_image)
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
        self.socketio.emit("temperature", {"value": float(message.data)})

    def _on_humidity(self, message):
        self.socketio.emit("humidity", {"value": float(message.data)})

    def _on_gas_reading(self, message):
        self.socketio.emit("gas_reading", {"value": float(message.data)})

    def _on_gas_status(self, message):
        self.socketio.emit("gas_status", {"value": int(message.data)})

    def _on_left_distance(self, message):
        self.socketio.emit("left_distance", {"value": float(message.data)})

    def _on_right_distance(self, message):
        self.socketio.emit("right_distance", {"value": float(message.data)})

    def _on_speed_mode(self, message):
        self.socketio.emit("speed_mode", {"value": int(message.data)})

    def _on_motor_left(self, message):
        self.socketio.emit("motor_left", {"value": int(message.data)})

    def _on_motor_right(self, message):
        self.socketio.emit("motor_right", {"value": int(message.data)})

    def _on_status(self, message):
        self.socketio.emit("status", {"value": message.data})

    def _on_joystick(self, message):
        self.socketio.emit("joystick_values", {"axes": list(message.axes), "buttons": list(message.buttons)})

    def run_web_server(self):
        self.socketio.run(self.app, host="0.0.0.0", port=self.web_port, allow_unsafe_werkzeug=True)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    web_thread = threading.Thread(target=node.run_web_server, daemon=True)
    web_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
