import time
import threading
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from drillpulse_autonomous.utils import log_colored

class CameraStream:
    """
    Manages camera image acquisition, frame buffering, FPS calculation,
    watchdog timer, and fallback simulation/webcam stream.
    """
    def __init__(self, node, topic="/camera/image_raw", target_width=640, target_height=480, fps_target=20, simulation=False, webcam_id=0):
        self.node = node
        self.topic = topic
        self.target_width = target_width
        self.target_height = target_height
        self.fps_target = fps_target
        self.simulation = simulation
        self.webcam_id = webcam_id

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        self.is_connected = False
        self.last_frame_time = 0.0
        self.frame_count = 0
        self.current_fps = 0.0
        self.fps_calc_time = time.monotonic()
        
        self.cap = None

        if self.node is not None:
            self.subscription = self.node.create_subscription(
                Image,
                self.topic,
                self._image_callback,
                1
            )
            log_colored("CAMERA", f"Subscribed to camera topic: {self.topic}", self.node.get_logger())
        
        if self.simulation:
            self._start_simulation_stream()

    def _image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if cv_img is not None:
                resized = cv2.resize(cv_img, (self.target_width, self.target_height))
                with self.frame_lock:
                    self.latest_frame = resized
                    self.last_frame_time = time.monotonic()
                    self.is_connected = True
                    self._update_fps()
        except Exception as e:
            if self.node:
                log_colored("CAMERA", f"CvBridge conversion error: {e}", self.node.get_logger())

    def _update_fps(self):
        self.frame_count += 1
        now = time.monotonic()
        elapsed = now - self.fps_calc_time
        if elapsed >= 1.0:
            self.current_fps = round(self.frame_count / elapsed, 1)
            self.frame_count = 0
            self.fps_calc_time = now

    def _start_simulation_stream(self):
        def webcam_worker():
            log_colored("CAMERA", f"Starting simulation mode webcam stream (device {self.webcam_id})...", getattr(self.node, 'get_logger', lambda: None)())
            self.cap = cv2.VideoCapture(self.webcam_id)
            if not self.cap.isOpened():
                log_colored("CAMERA", "Webcam device could not be opened. Generating synthetic test pattern stream.", getattr(self.node, 'get_logger', lambda: None)())
            
            while True:
                now = time.monotonic()
                if self.is_connected and (now - self.last_frame_time < 2.0):
                    # Real ROS2 camera active, sleep worker
                    time.sleep(0.5)
                    continue

                frame = None
                if self.cap and self.cap.isOpened():
                    ret, raw_frame = self.cap.read()
                    if ret:
                        frame = cv2.resize(raw_frame, (self.target_width, self.target_height))
                
                if frame is None:
                    # Generate synthetic corridor test image for simulation
                    frame = np.zeros((self.target_height, self.target_width, 3), dtype=np.uint8)
                    cv2.rectangle(frame, (50, 50), (self.target_width - 50, self.target_height - 50), (40, 40, 40), -1)
                    cv2.putText(frame, "SIMULATION CAMERA", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                with self.frame_lock:
                    self.latest_frame = frame
                    self.last_frame_time = time.monotonic()
                    self.is_connected = True
                    self._update_fps()
                
                time.sleep(1.0 / self.fps_target)

        thread = threading.Thread(target=webcam_worker, daemon=True)
        thread.start()

    def get_latest_frame(self):
        """Returns the latest buffered OpenCV frame and connection status."""
        now = time.monotonic()
        with self.frame_lock:
            # Watchdog check: if no frame received for > 2 seconds, mark disconnected
            if now - self.last_frame_time > 2.0 and not self.simulation:
                self.is_connected = False
            
            return self.latest_frame, self.is_connected, self.current_fps
