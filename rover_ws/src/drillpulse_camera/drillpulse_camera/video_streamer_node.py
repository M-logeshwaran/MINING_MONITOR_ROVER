#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Adaptive Video Streamer Node (WiFi / WiFi-HaLow)
# Platform: Ubuntu 24.04 LTS + ROS 2 Jazzy + Python 3.12
#
# Adaptive Behavior:
#   - GOOD LINK: 20 FPS, 640x480, JPEG Quality 75
#   - WEAK LINK: 5 FPS, 320x240, JPEG Quality 30 (bandwidth throttled)
#   - LOST / SUSPENDED: 0 FPS (video paused, zero CPU/bandwidth usage,
#     ensures zero interference with critical LoRa telemetry)
#   - RECONNECTED: Automatically restores normal optical/thermal stream
# ============================================================

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from drillpulse_msgs.msg import LinkStatus


class VideoStreamerNode(Node):
    def __init__(self):
        super().__init__('video_streamer_node')
        self.declare_parameter('transport_type', 'WIFI_HALOW')
        self.declare_parameter('rgb_device', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('adaptive_video_enabled', True)

        self.transport_type = str(self.get_parameter('transport_type').value).upper()
        self.base_width = int(self.get_parameter('frame_width').value)
        self.base_height = int(self.get_parameter('frame_height').value)
        self.base_fps = float(self.get_parameter('fps').value)
        self.sim_mode = bool(self.get_parameter('simulation_mode').value)
        self.adaptive_video = bool(self.get_parameter('adaptive_video_enabled').value)

        self.current_fps = self.base_fps
        self.current_width = self.base_width
        self.current_height = self.base_height
        self.jpeg_quality = 75
        self.video_suspended = False
        self.link_state = "GOOD"

        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.comp_pub = self.create_publisher(CompressedImage, '/camera/image_compressed', 10)
        self.comp_pub_alt = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self.thermal_pub = self.create_publisher(Image, '/thermal/image_raw', 10)
        self.thermal_comp_pub = self.create_publisher(CompressedImage, '/camera/thermal/compressed', 10)
        self.link_pub = self.create_publisher(LinkStatus, '/transport/video/link_status', 10)

        # Subscriptions for adaptive throttling
        self.create_subscription(LinkStatus, '/transport/video/link_status', self.on_video_link_status, 10)
        self.create_subscription(LinkStatus, '/rover/link_status', self.on_rover_link_status, 10)

        self.sim_step = 0
        self.last_frame_publish_time = time.time()
        self.timer = self.create_timer(0.05, self.adaptive_capture_loop)
        self.get_logger().info(f'Adaptive Video Streamer initialized (Transport: {self.transport_type}, Adaptive: {self.adaptive_video})')

    def on_video_link_status(self, msg: LinkStatus):
        self._update_link_profile(msg.connected, msg.signal_strength_rssi, msg.packet_loss_rate)

    def on_rover_link_status(self, msg: LinkStatus):
        # Fallback if separate video link status is not published
        if 'WIFI' in msg.link_type:
            self._update_link_profile(msg.connected, msg.signal_strength_rssi, msg.packet_loss_rate)

    def _update_link_profile(self, connected: bool, rssi: float, loss: float):
        if not self.adaptive_video:
            return

        if not connected or rssi < -110.0 or loss >= 0.80:
            if not self.video_suspended:
                self.get_logger().warn("Wi-Fi HaLow link LOST: Suspending video stream to prioritize critical LoRa telemetry.")
            self.video_suspended = True
            self.link_state = "LOST"
            self.current_fps = 0.0
        elif rssi < -85.0 or loss >= 0.35:
            if self.link_state != "WEAK":
                self.get_logger().info("Wi-Fi HaLow degraded: Throttling video to 5 FPS / low quality.")
            self.video_suspended = False
            self.link_state = "WEAK"
            self.current_fps = 5.0
            self.current_width = 320
            self.current_height = 240
            self.jpeg_quality = 30
        else:
            if self.video_suspended or self.link_state != "GOOD":
                self.get_logger().info("Wi-Fi HaLow restored: Resuming high-resolution 20 FPS video stream.")
            self.video_suspended = False
            self.link_state = "GOOD"
            self.current_fps = self.base_fps
            self.current_width = self.base_width
            self.current_height = self.base_height
            self.jpeg_quality = 75

    def adaptive_capture_loop(self):
        if self.video_suspended or self.current_fps <= 0.0:
            return

        now_sec = time.time()
        interval = 1.0 / self.current_fps
        if (now_sec - self.last_frame_publish_time) < interval:
            return
        self.last_frame_publish_time = now_sec

        self.capture_and_publish()

    def capture_and_publish(self):
        now = self.get_clock().now()
        self.sim_step += 1
        w, h = self.current_width, self.current_height

        rgb_frame = np.zeros((h, w, 3), dtype=np.uint8)
        cv2.rectangle(rgb_frame, (20, 20), (w - 20, h - 20), (45, 45, 45), 2)
        cv2.putText(rgb_frame, "DRILLPULSE OPTICAL RGB", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1)
        cv2.putText(rgb_frame, f"Link: {self.link_state} ({self.current_fps:.0f} FPS)", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # Thermal
        therm = np.zeros((h, w), dtype=np.uint8)
        cx, cy = w // 2, h // 2
        cv2.circle(therm, (cx + 30, cy - 10), 30, 220, -1)
        therm = cv2.GaussianBlur(therm, (35, 35), 0)
        therm_color = cv2.applyColorMap(therm, cv2.COLORMAP_INFERNO)
        cv2.putText(therm_color, "THERMAL FLIR IR", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
        rgb_msg.header.stamp = now.to_msg()
        rgb_msg.header.frame_id = 'camera_link'
        self.rgb_pub.publish(rgb_msg)

        success, encoded = cv2.imencode('.jpg', rgb_frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if success:
            cmsg = CompressedImage()
            cmsg.header.stamp = now.to_msg()
            cmsg.header.frame_id = 'camera_link'
            cmsg.format = 'jpeg'
            cmsg.data = encoded.tobytes()
            self.comp_pub.publish(cmsg)
            self.comp_pub_alt.publish(cmsg)

        tmsg = self.bridge.cv2_to_imgmsg(therm_color, encoding='bgr8')
        tmsg.header.stamp = now.to_msg()
        tmsg.header.frame_id = 'thermal_link'
        self.thermal_pub.publish(tmsg)

        success_t, encoded_t = cv2.imencode('.jpg', therm_color, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if success_t:
            tcmsg = CompressedImage()
            tcmsg.header.stamp = now.to_msg()
            tcmsg.header.frame_id = 'thermal_link'
            tcmsg.format = 'jpeg'
            tcmsg.data = encoded_t.tobytes()
            self.thermal_comp_pub.publish(tcmsg)

        link = LinkStatus()
        link.header.stamp = now.to_msg()
        link.link_type = f"{self.transport_type} [{self.link_state}]"
        link.connected = not self.video_suspended
        link.signal_strength_rssi = -60.0 if self.link_state == "GOOD" else (-92.0 if self.link_state == "WEAK" else -120.0)
        link.latency_ms = 25.0 if self.link_state == "GOOD" else (120.0 if self.link_state == "WEAK" else 999.0)
        self.link_pub.publish(link)


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
