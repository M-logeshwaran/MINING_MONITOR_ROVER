
#!/usr/bin/env python3

import time
import threading

import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge


# ============================================================
# ESP32-CAM CONFIGURATION
# ============================================================
#
# Put the IP address printed by your ESP32-CAM here.
#
# Example:
#
# ESP32_CAM_IP = "10.246.202.96"
#
# The node automatically creates:
#
# http://10.246.202.96/stream
#
# ============================================================

ESP32_CAM_IP = "10.36.163.96"


# ============================================================
# CAMERA STREAM
# ============================================================

ESP32_CAM_STREAM_URL = (
    "http://" +
    ESP32_CAM_IP +
    "/stream"
)


# ============================================================
# CAMERA SETTINGS
# ============================================================

CAMERA_RECONNECT_DELAY = 2.0

CAMERA_FPS_LIMIT = 20.0


# ============================================================
# ROS2 NODE
# ============================================================

class ESP32CamNode(Node):

    def __init__(self):

        super().__init__(
            "drillpulse_esp32_cam"
        )


        # ====================================================
        # LIVE FEED PUBLISHER
        # ====================================================

        self.live_feed_pub = self.create_publisher(
            Image,
            "/live_feed",
            10
        )


        # ====================================================
        # STATUS PUBLISHER
        # ====================================================

        self.status_pub = self.create_publisher(
            String,
            "/drillpulse/camera_status",
            10
        )


        # ====================================================
        # CV BRIDGE
        # ====================================================

        self.bridge = CvBridge()


        # ====================================================
        # CAMERA STATE
        # ====================================================

        self.camera_running = True

        self.camera_connected = False

        self.camera_frame_count = 0

        self.last_camera_frame_time = 0.0


        # ====================================================
        # CAMERA THREAD
        # ====================================================
        #
        # Camera runs independently.
        #
        # It does NOT access:
        #
        # /dev/rfcomm0
        #
        # It does NOT communicate with HC-05.
        #
        # It does NOT control the rover.
        #
        # ====================================================

        self.camera_thread = threading.Thread(
            target=self.camera_loop,
            daemon=True
        )

        self.camera_thread.start()


        # ====================================================
        # STATUS TIMER
        # ====================================================

        self.status_timer = self.create_timer(
            2.0,
            self.print_status
        )


        # ====================================================
        # STARTUP
        # ====================================================

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "       DRILLPULSE ESP32-CAM NODE"
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"Camera IP : {ESP32_CAM_IP}"
        )

        self.get_logger().info(
            f"Stream    : {ESP32_CAM_STREAM_URL}"
        )

        self.get_logger().info(
            "ROS Topic : /live_feed"
        )

        self.get_logger().info(
            "FPS Limit : 20"
        )

        self.get_logger().info(
            "=========================================="
        )


    # ========================================================
    # CAMERA LOOP
    # ========================================================

    def camera_loop(self):

        while (
            self.camera_running
            and rclpy.ok()
        ):

            capture = None


            try:

                # ------------------------------------------------
                # CONNECT
                # ------------------------------------------------

                self.get_logger().info(
                    f"Connecting to ESP32-CAM: "
                    f"{ESP32_CAM_STREAM_URL}"
                )


                capture = cv2.VideoCapture(
                    ESP32_CAM_STREAM_URL
                )


                # ------------------------------------------------
                # REDUCE OPENCV BUFFER
                # ------------------------------------------------
                #
                # This helps reduce old/stale frames.
                #
                # ------------------------------------------------

                try:

                    capture.set(
                        cv2.CAP_PROP_BUFFERSIZE,
                        1
                    )

                except Exception:

                    pass


                # ------------------------------------------------
                # CONNECTION CHECK
                # ------------------------------------------------

                if not capture.isOpened():

                    self.camera_connected = False

                    self.publish_status(
                        "ESP32-CAM DISCONNECTED"
                    )

                    self.get_logger().warning(
                        "ESP32-CAM connection failed"
                    )


                    if capture:

                        capture.release()


                    time.sleep(
                        CAMERA_RECONNECT_DELAY
                    )

                    continue


                # ------------------------------------------------
                # CONNECTED
                # ------------------------------------------------

                self.camera_connected = True

                self.publish_status(
                    "ESP32-CAM CONNECTED"
                )

                self.get_logger().info(
                    "ESP32-CAM CONNECTED"
                )


                # ------------------------------------------------
                # FPS LIMIT
                # ------------------------------------------------

                frame_interval = (
                    1.0 /
                    CAMERA_FPS_LIMIT
                )

                last_frame_time = 0.0


                # =================================================
                # FRAME LOOP
                # =================================================

                while (
                    self.camera_running
                    and rclpy.ok()
                ):

                    success, frame = (
                        capture.read()
                    )


                    # ------------------------------------------------
                    # FRAME READ FAILED
                    # ------------------------------------------------

                    if not success:

                        self.get_logger().warning(
                            "ESP32-CAM frame read failed"
                        )

                        break


                    if frame is None:

                        continue


                    # ------------------------------------------------
                    # FPS CONTROL
                    # ------------------------------------------------

                    now = time.monotonic()


                    if (
                        now - last_frame_time
                        < frame_interval
                    ):

                        continue


                    last_frame_time = now


                    # =================================================
                    # CONVERT OPENCV -> ROS IMAGE
                    # =================================================

                    try:

                        ros_image = (
                            self.bridge.cv2_to_imgmsg(
                                frame,
                                encoding="bgr8"
                            )
                        )


                        # ------------------------------------------------
                        # ROS TIMESTAMP
                        # ------------------------------------------------

                        ros_image.header.stamp = (
                            self.get_clock()
                            .now()
                            .to_msg()
                        )


                        # ------------------------------------------------
                        # FRAME ID
                        # ------------------------------------------------

                        ros_image.header.frame_id = (
                            "esp32_cam"
                        )


                        # ------------------------------------------------
                        # PUBLISH
                        # ------------------------------------------------

                        self.live_feed_pub.publish(
                            ros_image
                        )


                        self.camera_frame_count += 1

                        self.last_camera_frame_time = (
                            time.monotonic()
                        )


                    except Exception as exc:

                        self.get_logger().warning(
                            "Failed to publish camera frame: "
                            f"{exc}"
                        )


                # =================================================
                # CAMERA DISCONNECTED
                # =================================================

                self.camera_connected = False

                self.publish_status(
                    "ESP32-CAM DISCONNECTED"
                )


                if capture:

                    capture.release()


                self.get_logger().warning(
                    "ESP32-CAM stream disconnected"
                )


            except Exception as exc:

                self.camera_connected = False

                self.publish_status(
                    "ESP32-CAM ERROR"
                )


                self.get_logger().warning(
                    f"Camera error: {exc}"
                )


                if capture:

                    try:

                        capture.release()

                    except Exception:

                        pass


            # ====================================================
            # AUTOMATIC RECONNECT
            # ====================================================

            if self.camera_running:

                time.sleep(
                    CAMERA_RECONNECT_DELAY
                )


    # ========================================================
    # STATUS
    # ========================================================

    def publish_status(
        self,
        text
    ):

        msg = String()

        msg.data = text

        self.status_pub.publish(msg)


    # ========================================================
    # TERMINAL STATUS
    # ========================================================

    def print_status(self):

        if self.camera_connected:

            if self.camera_frame_count > 0:

                age = (
                    time.monotonic()
                    -
                    self.last_camera_frame_time
                )

                self.get_logger().info(
                    f"CAMERA: CONNECTED | "
                    f"Frames: {self.camera_frame_count} | "
                    f"Last frame: {age:.2f}s | "
                    f"Topic: /live_feed"
                )

            else:

                self.get_logger().info(
                    "CAMERA: CONNECTED | "
                    "Waiting for frames"
                )

        else:

            self.get_logger().warning(
                "CAMERA: DISCONNECTED | "
                "Retrying..."
            )


    # ========================================================
    # SHUTDOWN
    # ========================================================

    def destroy_node(self):

        self.get_logger().info(
            "Shutting down ESP32-CAM node..."
        )


        self.camera_running = False


        if (
            hasattr(
                self,
                "camera_thread"
            )
            and
            self.camera_thread.is_alive()
        ):

            self.camera_thread.join(
                timeout=2.0
            )


        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = ESP32CamNode()

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        if node is not None:

            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
