#!/usr/bin/env python3
"""
DRILLPULSE – High-Performance Unified Rover Controller V2
==========================================================

Preserves the external behavior/protocol of drillpulse_rover_rollback_unified.py
while separating high-priority control from odometry, ROS publishing and the
Matplotlib GUI.

IMPORTANT HONESTY NOTE
----------------------
This prototype has NO physical wheel encoders. Position, heading, RPM and
rotations are software-predicted values (Virtual Encoder Simulation / Predicted
Odometry). They are suitable for a prototype demonstration, not for
centimeter-level mine navigation.

Controls preserved from the source:
    Joystick / D-pad:
        UP       = Forward
        DOWN     = Backward
        LEFT     = Left
        RIGHT    = Right

    Pygame Button 0 = speed mode:
        NORMAL -> TURBO -> SLOW -> NORMAL

    Pygame Button 2 = automatic rollback:
        press once = start rollback
        press again = cancel rollback

    Autonomous point-to-point:
        left-click map = select destination
        first available unused joystick button = start/cancel autonomous
        keyboard A = start/cancel autonomous
        right-click map = clear destination

Map keyboard:
    R = reset
    F = follow rover
    G = grid on/off
    H = heading on/off
    S = save map
    Q = quit

Bluetooth protocol is unchanged:
    /dev/rfcomm0 @ 9600 baud
    CMD,X,Y,SPEED\\n

    Arduino telemetry:
        S,TEMP,HUMIDITY,MQ4_ANALOG,MQ4_DIGITAL,LEFT_DISTANCE,RIGHT_DISTANCE\\n
        Legacy labeled telemetry is also accepted.

Motor mapping is unchanged:
    M1/M2 = left side, M3/M4 = right side
    Forward  = all +
    Backward = all -
    Right    = left +, right -
    Left     = left -, right +
"""

import math
import threading
import time
from collections import deque

import pygame
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int8, Int32, Float32, String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.widgets import Button


class BluetoothManager:
    """Single-owner Bluetooth manager with independent RX/TX threads.

    RX continuously parses Arduino sensor/status lines. TX sends only the
    latest movement command at a fixed keepalive rate so the control loop
    never blocks on serial I/O. Failed ports are closed before reconnecting.
    """

    def __init__(self, logger, device='/dev/rfcomm0', baud=9600):
        self.logger = logger
        self.device = device
        self.baud = baud
        self.serial = None
        self.connected = False
        self.lock = threading.RLock()
        self.connect_lock = threading.Lock()
        self.running = True
        self.last_error = ''
        self.last_rx_time = 0.0
        self.rx_buffer = ''
        self._latest_command = None
        self._priority_command = None
        self._tx_event = threading.Event()
        self.rx_callback = None
        self.tx_sequence = 0

        # 9600-baud HC-05 + Arduino SoftwareSerial:
        # keep movement traffic conservative while retaining immediate
        # direction changes and a watchdog keepalive.
        self.MIN_TX_INTERVAL = 0.045
        self.last_tx_time = 0.0

        self.rx_thread = threading.Thread(
            target=self._rx_loop, name='drillpulse-bt-rx', daemon=True
        )
        self.tx_thread = threading.Thread(
            target=self._tx_loop, name='drillpulse-bt-tx', daemon=True
        )
        self.rx_thread.start()
        self.tx_thread.start()

    def set_rx_callback(self, callback):
        self.rx_callback = callback

    def discard_pending_rx(self):
        with self.lock:
            self.rx_buffer = ''
            if self.serial is not None:
                try:
                    self.serial.reset_input_buffer()
                except Exception:
                    pass

    def _close_locked(self):
        port = self.serial
        self.serial = None
        self.connected = False
        if port is not None:
            try:
                port.close()
            except Exception:
                pass

    def close(self):
        self.running = False
        self._tx_event.set()
        with self.lock:
            self._close_locked()
        for t in (self.rx_thread, self.tx_thread):
            if t.is_alive():
                t.join(timeout=1.0)

    def connect(self):
        if not self.running:
            return False
        with self.connect_lock:
            with self.lock:
                if self.serial is not None and self.connected:
                    return True
                self._close_locked()
                try:
                    self.serial = serial.Serial(
                        port=self.device, baudrate=self.baud,
                        timeout=0.01, write_timeout=0.08
                    )
                    try:
                        self.serial.reset_input_buffer()
                    except Exception:
                        pass
                    self.rx_buffer = ''
                    self.connected = True
                    self.last_error = ''
                    self.logger.info(
                        f'HC-05 connected: {self.device} @ {self.baud}'
                    )
                    return True
                except Exception as exc:
                    self.last_error = str(exc)
                    self.connected = False
                    self.serial = None
                    return False

    def queue_command(self, packet):
        """Replace the previous movement packet and wake TX immediately."""
        with self.lock:
            self._latest_command = packet
        self._tx_event.set()

    def queue_priority_command(self, packet):
        """Send a mode packet before the next movement packet."""
        with self.lock:
            self._priority_command = packet
        self._tx_event.set()

    def send(self, packet):
        """Synchronous one-shot packet for reset/stop/status commands only."""
        with self.lock:
            if self.serial is None or not self.connected:
                return False
            try:
                self.serial.write(packet.encode('ascii'))
                self.last_tx_time = time.monotonic()
                return True
            except Exception as exc:
                self.last_error = str(exc)
                self.logger.warning(f'Bluetooth TX error: {exc}')
                self._close_locked()
                return False

    def safe_stop(self):
        self.queue_command('CMD,0.000,0.000,1\n')

    def _tx_loop(self):
        """Low-latency event-driven TX.

        IMPORTANT: This thread sends ONLY packets explicitly queued by the
        control loop. It does NOT retransmit the last packet by itself.
        The control loop decides when a moving keepalive is needed.

        This prevents the old failure mode where STOP (0,0) was transmitted
        continuously at 20 Hz while the rover was idle, starving Arduino's
        SoftwareSerial link and interfering with sensor telemetry.
        """
        last_sent_packet = None

        while self.running and rclpy.ok():
            if not self.connected:
                self.connect()
                self._tx_event.wait(0.05)
                self._tx_event.clear()
                continue

            packet = None
            priority_packet = None
            with self.lock:
                priority_packet = self._priority_command
                current = self._latest_command
                if priority_packet is None and current is not None and current != last_sent_packet:
                    packet = current

            if priority_packet is not None and self.send(priority_packet):
                with self.lock:
                    if self._priority_command == priority_packet:
                        self._priority_command = None
                self._tx_event.set()

            if packet is not None and self.send(packet):
                last_sent_packet = packet

            # Event-driven: a newly queued direction/speed/keepalive packet
            # wakes this thread immediately.
            self._tx_event.wait()
            self._tx_event.clear()

    def _rx_loop(self):
        while self.running and rclpy.ok():
            if not self.connected:
                self.connect()
                time.sleep(0.25)
                continue
            try:
                with self.lock:
                    port = self.serial
                    waiting = port.in_waiting if port is not None else 0
                    data = port.read(waiting) if port is not None and waiting > 0 else b''
                if data:
                    self.last_rx_time = time.monotonic()
                    self.rx_buffer += data.decode('utf-8', errors='ignore')
                    if len(self.rx_buffer) > 4096:
                        self.rx_buffer = ''
                        with self.lock:
                            if self.serial is not None:
                                try:
                                    self.serial.reset_input_buffer()
                                except Exception:
                                    pass
                    while '\n' in self.rx_buffer:
                        line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                        line = line.strip()
                        if line and self.rx_callback is not None:
                            try:
                                self.rx_callback(line)
                            except Exception as exc:
                                self.logger.warning(f'BT RX callback error: {exc}')
                else:
                    time.sleep(0.005)
            except (serial.SerialException, OSError) as exc:
                self.last_error = str(exc)
                self.logger.warning(f'Bluetooth RX error: {exc}')
                with self.lock:
                    self._close_locked()
                time.sleep(0.25)
            except Exception as exc:
                self.logger.warning(f'Bluetooth RX unexpected error: {exc}')
                with self.lock:
                    self._close_locked()
                time.sleep(0.25)


class MovementSegmentRecorder:
    """Compact movement history: one object per continuous command segment."""

    def __init__(self, lock, min_duration=0.02):
        self.lock = lock
        self.min_duration = min_duration
        self.current = None
        self.segments = []

    def update(self, direction, mode, now):
        """Start/continue/finish a segment based on real monotonic time."""
        with self.lock:
            if direction == (0, 0):
                self._finish_locked(now)
                return

            dx, dy = direction
            if (
                self.current is not None
                and self.current['x'] == dx
                and self.current['y'] == dy
                and self.current['mode'] == mode
            ):
                return

            self._finish_locked(now)
            self.current = {
                'x': int(dx),
                'y': int(dy),
                'mode': int(mode),
                'duration': 0.0,
                'started_at': now,
            }

    def finish(self, now):
        with self.lock:
            self._finish_locked(now)

    def _finish_locked(self, now):
        if self.current is None:
            return
        duration = max(0.0, now - self.current['started_at'])
        if duration >= self.min_duration:
            self.segments.append({
                'x': self.current['x'],
                'y': self.current['y'],
                'mode': self.current['mode'],
                'duration': duration,
            })
        self.current = None

    def snapshot(self):
        with self.lock:
            result = [dict(s) for s in self.segments]
            if self.current is not None:
                result.append(dict(self.current))
            return result

    def clear(self):
        with self.lock:
            self.current = None
            self.segments.clear()

    def count(self):
        with self.lock:
            return len(self.segments) + (1 if self.current else 0)


class DrillPulseRover(Node):
    def __init__(self):
        super().__init__('drillpulse_rover')

        # =============================================================
        # CONFIGURATION — kept compatible with V1
        # =============================================================
        self.BT_DEVICE = '/dev/rfcomm0'
        self.BT_BAUD = 9600
        self.MAX_PWM = 200
        self.MAX_SPEED = 0.50
        self.WHEEL_DIAMETER = 0.065
        self.WHEEL_BASE = 0.30

        self.SLOW_MULTIPLIER = 0.40
        self.NORMAL_MULTIPLIER = 0.70
        self.TURBO_MULTIPLIER = 1.00

        self.DEADZONE = 0.15
        self.COMMAND_TIMEOUT = 0.50
        self.CONTROL_HZ = 50.0
        self.ODOMETRY_HZ = 50.0
        self.PATH_HZ = 50.0
        self.GUI_HZ = 60.0
        self.TELEMETRY_HZ = 10.0
        self.ROLLBACK_HZ = 50.0
        self.KEEPALIVE_PERIOD = 0.35

        self.SPEED_BUTTON = 0
        self.ROLLBACK_BUTTON = 2  # verified from source; pygame is zero-based
        # Autonomous uses the first physically available unused pygame button.
        # Button 0 and Button 2 remain reserved for speed/rollback.
        self.AUTONOMOUS_BUTTON = None
        self.FULL_CONTROL_BUTTON = None

        self.AUTO_HEADING_TOLERANCE = math.radians(6.0)
        self.AUTO_HEADING_REACQUIRE = math.radians(11.0)
        self.AUTO_TARGET_TOLERANCE = 0.08
        self.AUTO_SLOW_DISTANCE = 0.30
        self.AUTO_CONTROL_HZ = 25.0
        self.AUTO_SLOW_MODE = 0

        self.MAX_TRAJECTORY_POINTS = 2000
        self.MAX_ROLLBACK_TRAIL_POINTS = 2000
        self.ROLLBACK_FINISH_DISTANCE = 0.10

        # =============================================================
        # SHARED STATE
        # =============================================================
        self.lock = threading.RLock()
        self.shutdown_event = threading.Event()

        self.speed_mode = 1
        self.joy_x = 0.0
        self.joy_y = 0.0
        self.last_command_time = time.monotonic()
        self.last_control_time = time.monotonic()
        self.last_sent_x = None
        self.last_sent_y = None
        self.last_sent_speed = None
        self.last_keepalive = 0.0

        # =============================================================
        # ARDUINO SENSOR STATE — updated by dedicated Bluetooth RX thread
        # =============================================================
        self.temperature = float('nan')
        self.humidity = float('nan')
        self.mq4_analog = -1
        self.mq4_digital = -1
        self.left_distance = -1.0
        self.right_distance = -1.0
        self.sensor_packet_count = 0
        self.invalid_sensor_packets = 0
        self.last_sensor_time = 0.0
        self.last_arduino_status = 'WAITING'

        self.rollback_active = False
        self.rollback_status = 'EXPLORATION'
        self.rollback_progress = 0.0
        self.rollback_distance_remaining = 0.0
        self.rollback_index = -1
        self.rollback_segment_elapsed = 0.0
        self.rollback_last_time = time.monotonic()
        self.rollback_trail = deque(maxlen=self.MAX_ROLLBACK_TRAIL_POINTS)
        self.rollback_path_points = []
        self.rollback_path_cursor = -1
        self.rollback_final_correction = False

        # =============================================================
        # AUTONOMOUS POINT-TO-POINT STATE
        # =============================================================
        self.autonomous_active = False
        self.autonomous_status = 'MANUAL'
        self.autonomous_target = None  # (x, y) in the virtual odom/map frame
        self.autonomous_distance_remaining = 0.0
        self.autonomous_heading_error = 0.0
        self.last_autonomous_button = 0
        self.full_control_active = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_distance = 0.0
        self.m1_rotations = 0.0
        self.m2_rotations = 0.0
        self.m3_rotations = 0.0
        self.m4_rotations = 0.0
        self.m1_rpm = 0.0
        self.m2_rpm = 0.0
        self.m3_rpm = 0.0
        self.m4_rpm = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_odom_time = time.monotonic()

        self.exploration_points = deque(
            [(0.0, 0.0, 0.0)],
            maxlen=self.MAX_TRAJECTORY_POINTS,
        )

        self.recorder = MovementSegmentRecorder(self.lock)

        self.follow_rover = True
        self.show_grid = True
        self.show_heading = True
        self.gui_alive = True

        # =============================================================
        # HARDWARE INITIALIZATION
        # =============================================================
        self.bt = BluetoothManager(self.get_logger(), self.BT_DEVICE, self.BT_BAUD)

        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.controller_connected = False
        self.last_speed_button = 0
        self.last_rollback_button = 0
        self.last_full_control_button = 0

        try:
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.controller_connected = True
                self.get_logger().info(
                    f'Joystick: {self.joystick.get_name()} | '
                    f'Axes: {self.joystick.get_numaxes()} | '
                    f'Buttons: {self.joystick.get_numbuttons()}'
                )
                button_count = self.joystick.get_numbuttons()
                if button_count <= self.ROLLBACK_BUTTON:
                    self.get_logger().warning(
                        'Joystick has fewer than 3 buttons; Button 2 rollback unavailable.'
                    )
                # Option B: keep Button 0 = speed and Button 2 = rollback.
                # Pick the first actually available unused button for autonomous.
                reserved = {self.SPEED_BUTTON, self.ROLLBACK_BUTTON}
                for idx in range(button_count):
                    if idx not in reserved:
                        self.AUTONOMOUS_BUTTON = idx
                        break
                if self.AUTONOMOUS_BUTTON is not None:
                    self.get_logger().info(
                        f'Button {self.AUTONOMOUS_BUTTON} = AUTONOMOUS POINT-TO-POINT'
                    )
                else:
                    self.get_logger().warning(
                        'No unused joystick button available for autonomous mode. '
                        'Use map click + keyboard A instead.'
                    )
                reserved.add(self.AUTONOMOUS_BUTTON)
                for idx in range(button_count):
                    if idx not in reserved:
                        self.FULL_CONTROL_BUTTON = idx
                        break
                if self.FULL_CONTROL_BUTTON is not None:
                    self.get_logger().info(
                        f'Button {self.FULL_CONTROL_BUTTON} = FULL CONTROL'
                    )
                else:
                    self.get_logger().warning(
                        'No unused joystick button available for full control.'
                    )
            else:
                self.get_logger().warning(
                    'No USB joystick detected at startup. Control thread will keep trying.'
                )
        except Exception as exc:
            self.get_logger().error(f'Pygame joystick initialization failed: {exc}')

        # =============================================================
        # ROS INTERFACES — same topic/service names as source
        # =============================================================
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.odom_group = MutuallyExclusiveCallbackGroup()
        self.telemetry_group = MutuallyExclusiveCallbackGroup()
        self.reset_group = MutuallyExclusiveCallbackGroup()
        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.joy_pub = self.create_publisher(Joy, '/joystick_value', 10)
        self.speed_pub = self.create_publisher(Int8, '/speed_mode', 10)
        self.motor_left_pub = self.create_publisher(Int32, '/drillpulse/motor_left', 10)
        self.motor_right_pub = self.create_publisher(Int32, '/drillpulse/motor_right', 10)
        self.odom_pub = self.create_publisher(Odometry, '/virtual_odom', 10)
        self.path_pub = self.create_publisher(Path, '/virtual_path', 10)
        self.rollback_status_pub = self.create_publisher(
            String, '/rollback_status', 10
        )
        self.autonomous_status_pub = self.create_publisher(
            String, '/autonomous_status', state_qos
        )
        self.full_control_pub = self.create_publisher(
            Bool, '/full_control', state_qos
        )
        full_control_msg = Bool()
        full_control_msg.data = self.full_control_active
        self.full_control_pub.publish(full_control_msg)
        speed_msg = Int8()
        speed_msg.data = self.speed_mode
        self.speed_pub.publish(speed_msg)
        autonomous_status_msg = String()
        autonomous_status_msg.data = self.autonomous_status
        self.autonomous_status_pub.publish(autonomous_status_msg)

        self.temperature_pub = self.create_publisher(Float32, '/drillpulse/temperature', 10)
        self.humidity_pub = self.create_publisher(Float32, '/drillpulse/humidity', 10)
        self.gas_pub = self.create_publisher(Int32, '/drillpulse/gas', 10)
        self.gas_status_pub = self.create_publisher(Int32, '/drillpulse/gas_status', 10)
        self.left_distance_pub = self.create_publisher(Float32, '/drillpulse/left_distance', 10)
        self.right_distance_pub = self.create_publisher(Float32, '/drillpulse/right_distance', 10)
        self.status_pub = self.create_publisher(String, '/drillpulse/status', 10)

        self.bt.set_rx_callback(self.process_bluetooth_message)

        self.sensor_status_timer = self.create_timer(1.0, self.print_sensor_terminal)

        self.reset_service = self.create_service(
            Trigger,
            '/reset_virtual_odom',
            self.reset_callback,
            callback_group=self.reset_group,
        )

        # Path message is bounded. It is only published at PATH_HZ.
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.odometry_timer = self.create_timer(
            1.0 / self.ODOMETRY_HZ,
            self.odometry_update,
            callback_group=self.odom_group,
        )
        self.path_publish_timer = self.create_timer(
            1.0 / self.PATH_HZ,
            self.publish_path,
            callback_group=self.telemetry_group,
        )
        self.rollback_status_timer = self.create_timer(
            1.0 / self.TELEMETRY_HZ,
            self.publish_rollback_status,
            callback_group=self.telemetry_group,
        )

        # =============================================================
        # MATPLOTLIB — created once and touched only by main GUI thread
        # =============================================================
        self.setup_map_ui()

        # High-priority control loop is a dedicated thread. It owns pygame
        # polling, button edge detection, rollback timing and Bluetooth writes.
        self.control_thread = threading.Thread(
            target=self.control_loop,
            name='drillpulse-control',
            daemon=True,
        )
        self.control_thread.start()

        self.get_logger().info('==================================================')
        self.get_logger().info('DRILLPULSE HIGH-PERFORMANCE UNIFIED V3')
        self.get_logger().info('Control=50Hz | BT TX=20Hz | BT RX=dedicated | Odom=50Hz | GUI=60Hz')
        self.get_logger().info(
            'Button 0 = SPEED | Button 2 = ROLLBACK | '
            'unused buttons = AUTONOMOUS then FULL CONTROL'
        )
        self.get_logger().info('Sensors: TEMP/HUMIDITY/MQ4/LEFT+RIGHT ULTRASONIC')
        self.get_logger().info('Camera remains independent on /live_feed')
        self.get_logger().info('==================================================')

    # =============================================================
    # JOYSTICK / CONTROL
    # =============================================================

    @staticmethod
    def clamp(value):
        return max(-1.0, min(1.0, float(value)))

    def deadzone(self, value, zone=None):
        zone = self.DEADZONE if zone is None else zone
        value = float(value)
        if abs(value) < zone:
            return 0.0
        return value

    def acquire_joystick(self):
        try:
            if pygame.joystick.get_count() <= 0:
                self.controller_connected = False
                return False
            if self.joystick is None or not self.joystick.get_init():
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
            self.controller_connected = True
            return True
        except Exception as exc:
            self.controller_connected = False
            self.get_logger().warning(f'Joystick unavailable: {exc}')
            return False

    def get_joystick_xy(self):
        # pygame event pumping is intentionally in the control thread only.
        pygame.event.pump()
        if self.joystick is None:
            return 0.0, 0.0

        try:
            analog_x = self.deadzone(self.joystick.get_axis(0))
            analog_y = self.deadzone(-self.joystick.get_axis(1))
        except Exception as exc:
            self.controller_connected = False
            self.get_logger().warning(f'Joystick read error: {exc}')
            return 0.0, 0.0

        dpad_x = 0
        dpad_y = 0
        try:
            if self.joystick.get_numhats() > 0:
                dpad_x, dpad_y = self.joystick.get_hat(0)
        except Exception:
            pass

        # Preserve source behavior: D-pad overrides an analog axis when nonzero.
        x = float(dpad_x) if dpad_x != 0 else analog_x
        y = float(dpad_y) if dpad_y != 0 else analog_y
        return round(self.clamp(x), 3), round(self.clamp(y), 3)

    def read_buttons(self):
        if self.joystick is None:
            return 0, 0, 0, []
        try:
            n = self.joystick.get_numbuttons()
            buttons = [self.joystick.get_button(i) for i in range(n)]
            speed = buttons[self.SPEED_BUTTON] if n > self.SPEED_BUTTON else 0
            rollback = buttons[self.ROLLBACK_BUTTON] if n > self.ROLLBACK_BUTTON else 0
            auto = (
                buttons[self.AUTONOMOUS_BUTTON]
                if self.AUTONOMOUS_BUTTON is not None and n > self.AUTONOMOUS_BUTTON
                else 0
            )
            full_control = (
                buttons[self.FULL_CONTROL_BUTTON]
                if self.FULL_CONTROL_BUTTON is not None and n > self.FULL_CONTROL_BUTTON
                else 0
            )
            return speed, rollback, auto, full_control, buttons
        except Exception as exc:
            self.controller_connected = False
            self.get_logger().warning(f'Joystick button read error: {exc}')
            return 0, 0, 0, 0, []

    def control_loop(self):
        """Dedicated 50 Hz high-priority loop; GUI cannot block it."""
        period = 1.0 / self.CONTROL_HZ
        next_tick = time.monotonic()

        try:
            while not self.shutdown_event.is_set() and rclpy.ok():
                now = time.monotonic()

                if not self.acquire_joystick():
                    self._control_fail_safe(now)
                    self.shutdown_event.wait(min(period, 0.1))
                    next_tick = time.monotonic()
                    continue

                x, y = self.get_joystick_xy()
                speed_button, rollback_button, auto_button, full_control_button, buttons = self.read_buttons()

                if full_control_button and not self.last_full_control_button:
                    if not (self.rollback_active or self.autonomous_active):
                        self.set_full_control(not self.full_control_active)

                # Button 2 rising edge: start/cancel rollback.
                if rollback_button and not self.last_rollback_button:
                    if self.rollback_active:
                        self.cancel_rollback()
                    else:
                        self.cancel_autonomous('ROLLBACK REQUESTED')
                        self.start_rollback()

                # Option B: an actually available unused button starts/cancels
                # autonomous point-to-point mode.
                if auto_button and not self.last_autonomous_button:
                    if self.autonomous_active:
                        self.cancel_autonomous('AUTONOMOUS CANCELLED BY BUTTON')
                    else:
                        self.start_autonomous()

                self.last_rollback_button = rollback_button
                self.last_autonomous_button = auto_button

                if not self.rollback_active:
                    # Button 0 rising edge: NORMAL -> TURBO -> SLOW -> NORMAL.
                    if speed_button and not self.last_speed_button:
                        self.set_speed_mode((self.speed_mode + 1) % 3)

                    # Manual joystick input has priority over autonomous mode.
                    manual_direction = self.command_direction(x, y)
                    if manual_direction != (0, 0) and self.autonomous_active:
                        self.cancel_autonomous('MANUAL JOYSTICK OVERRIDE')

                    if self.autonomous_active:
                        self._autonomous_step(now)
                        self.publish_joy(self.joy_x, self.joy_y, buttons)
                    else:
                        with self.lock:
                            self.joy_x = x
                            self.joy_y = y
                            self.last_command_time = now

                        self._record_manual_command(x, y, now)
                        self._send_motion_if_needed(x, y, now, force=False)
                        self.publish_joy(x, y, buttons)
                else:
                    self._rollback_step(now)

                self.last_speed_button = speed_button
                self.last_control_time = now
                self.last_full_control_button = full_control_button

                next_tick += period
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    self.shutdown_event.wait(sleep_time)
                else:
                    next_tick = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f'CONTROL LOOP FAILURE: {exc}')
            self._control_fail_safe(time.monotonic())

    def _control_fail_safe(self, now):
        with self.lock:
            self.joy_x = 0.0
            self.joy_y = 0.0
            self.last_command_time = now
            self.recorder.finish(now)
        self._send_motion_if_needed(0.0, 0.0, now, force=True)

    def publish_joy(self, x, y, buttons):
        try:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [float(x), float(y)]
            msg.buttons = list(buttons)
            self.joy_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f'/joystick_value publish error: {exc}')

    # =============================================================
    # BLUETOOTH / COMMAND STATE
    # =============================================================

    def set_full_control(self, active):
        active = bool(active)
        if active == self.full_control_active:
            return
        self.full_control_active = active
        self.bt.discard_pending_rx()
        self._clear_sensor_state()
        self.bt.queue_priority_command(f'FULL,{1 if active else 0}\n')
        msg = Bool()
        msg.data = active
        self.full_control_pub.publish(msg)
        self.get_logger().info(
            f'FULL CONTROL: {"ON" if active else "OFF"}'
        )

    def publish_autonomous_status(self):
        msg = String()
        with self.lock:
            msg.data = self.autonomous_status
        self.autonomous_status_pub.publish(msg)

    def _send_motion_if_needed(self, x, y, now, force=False):
        """Queue one atomic CMD,X,Y,SPEED packet.

        Direction is cardinalized exactly as before. The speed mode is embedded
        in the same packet, so a speed change while moving cannot be lost behind
        a separate SPEED packet.
        """
        direction = self.command_direction(x, y)
        tx_x, tx_y = float(direction[0]), float(direction[1])
        speed = int(self.speed_mode)

        changed = (
            self.last_sent_x is None
            or tx_x != self.last_sent_x
            or tx_y != self.last_sent_y
            or self.last_sent_speed is None
            or speed != self.last_sent_speed
        )
        moving = (tx_x != 0.0 or tx_y != 0.0)
        keepalive_due = moving and ((now - self.last_keepalive) >= self.KEEPALIVE_PERIOD)

        # STOP is event-driven: send it once when the rover becomes stopped,
        # never repeatedly while the joystick remains centered.
        if force or changed or keepalive_due:
            self.bt.tx_sequence += 1
            seq = self.bt.tx_sequence
            packet = f'CMD,{seq},{tx_x:.3f},{tx_y:.3f},{speed}\n'
            self.bt.queue_command(packet)
            # INFO only for a state-change packet; keepalive packets are quiet.
            if changed or force:
                self.get_logger().info(
                    f'BT TX seq={seq} x={tx_x:.0f} y={tx_y:.0f} speed={speed}'
                )
            self.last_sent_x = tx_x
            self.last_sent_y = tx_y
            self.last_sent_speed = speed
            self.last_keepalive = now

    def send_speed_mode(self, mode=None, force=False):
        """Compatibility helper; speed is transported inside CMD."""
        if mode is not None:
            mode = int(mode) % 3
            with self.lock:
                self.speed_mode = mode
        if force:
            self.last_sent_speed = None
        with self.lock:
            x = self.joy_x
            y = self.joy_y
        self._send_motion_if_needed(x, y, time.monotonic(), force=force)

    def stop_rover(self):
        now = time.monotonic()
        with self.lock:
            self.joy_x = 0.0
            self.joy_y = 0.0
            self.last_command_time = now
            self.recorder.finish(now)
        self._send_motion_if_needed(0.0, 0.0, now, force=True)

    # =============================================================
    # ARDUINO BLUETOOTH RX / SENSOR DATA
    # =============================================================

    def _clear_sensor_state(self):
        with self.lock:
            self.temperature = float('nan')
            self.humidity = float('nan')
            self.mq4_analog = -1
            self.mq4_digital = -1
            self.left_distance = -1.0
            self.right_distance = -1.0
            self.sensor_packet_count = 0
            self.last_sensor_time = 0.0
            self.last_arduino_status = 'SENSORS_BLOCKED'

    def process_bluetooth_message(self, message):
        """Parse Arduino telemetry/status without blocking the control path.

        Supported sensor formats:

        1) Compact current Arduino format:
           S,TEMP,HUMIDITY,MQ4_ANALOG,MQ4_DIGITAL,LEFT_DISTANCE,RIGHT_DISTANCE

        2) Legacy labeled format:
           TEMP=...,HUMIDITY=...,MQ4_ANALOG=...,MQ4_DIGITAL=...,
           LEFT_DISTANCE=...,RIGHT_DISTANCE=...

        A malformed line is ignored instead of being published as sensor data.
        """
        message = message.strip()

        if not message:
            return

        if self.full_control_active:
            return

        # Current Arduino telemetry format.
        if message.startswith("S,"):
            if self.process_compact_sensor_data(message):
                return

        # Legacy telemetry format.
        if (
            message.startswith("TEMP=")
            and "HUMIDITY=" in message
            and "MQ4_ANALOG=" in message
        ):
            if self.process_sensor_data(message):
                return

        # Status / acknowledgements.
        self.last_arduino_status = message

        if message.startswith("ERROR,"):
            # Do not flood the console with malformed serial lines.
            return

        if message in (
            "DRILLPULSE ARDUINO READY",
            "HC-05 CONNECTED",
            "BAUD=9600",
            "READY",
        ):
            self.get_logger().info(f"Arduino: {message}")

        elif (
            message.startswith("SPEED:")
            or message.startswith("ACK,")
            or message in ("ARDUINO RESET: OK", "RESET_OK")
        ):
            self.get_logger().info(f"Arduino: {message}")

        try:
            msg = String()
            msg.data = message
            self.status_pub.publish(msg)
        except Exception:
            pass

    def _publish_sensor_values(
        self,
        temperature,
        humidity,
        gas,
        gas_status,
        left,
        right,
    ):
        """Store and publish one complete sensor sample."""
        if self.full_control_active:
            return False

        # -1.0 is the Arduino's valid 'no ultrasonic echo' value.
        # Temperature/humidity/gas must be finite.
        if not math.isfinite(temperature):
            return False
        if not math.isfinite(humidity):
            return False
        if not math.isfinite(left):
            return False
        if not math.isfinite(right):
            return False

        gas = int(gas)
        gas_status = int(gas_status)

        with self.lock:
            self.temperature = float(temperature)
            self.humidity = float(humidity)
            self.mq4_analog = gas
            self.mq4_digital = gas_status
            self.left_distance = float(left)
            self.right_distance = float(right)

            self.sensor_packet_count += 1
            self.last_sensor_time = time.monotonic()
            self.last_arduino_status = "SENSOR_OK"

        # ---- Temperature ----
        msg = Float32()
        msg.data = float(temperature)
        self.temperature_pub.publish(msg)

        # ---- Humidity ----
        msg = Float32()
        msg.data = float(humidity)
        self.humidity_pub.publish(msg)

        # ---- MQ-4 analog ----
        msg = Int32()
        msg.data = gas
        self.gas_pub.publish(msg)

        # ---- MQ-4 digital ----
        msg = Int32()
        msg.data = gas_status
        self.gas_status_pub.publish(msg)

        # ---- Left ultrasonic ----
        msg = Float32()
        msg.data = float(left)
        self.left_distance_pub.publish(msg)

        # ---- Right ultrasonic ----
        msg = Float32()
        msg.data = float(right)
        self.right_distance_pub.publish(msg)

        return True

    def process_compact_sensor_data(self, message):
        """Parse: S,TEMP,HUMIDITY,MQ4A,MQ4D,LEFT,RIGHT"""
        try:
            parts = [p.strip() for p in message.split(",")]

            # Exactly 7 fields are required:
            # S + 6 sensor values.
            if len(parts) != 7 or parts[0] != "S":
                self.invalid_sensor_packets += 1
                return False

            temperature = float(parts[1])
            humidity = float(parts[2])
            gas = int(float(parts[3]))
            gas_status = int(float(parts[4]))
            left = float(parts[5])
            right = float(parts[6])

            return self._publish_sensor_values(
                temperature,
                humidity,
                gas,
                gas_status,
                left,
                right,
            )

        except (ValueError, TypeError, IndexError):
            self.invalid_sensor_packets += 1
            return False
        except Exception as exc:
            self.invalid_sensor_packets += 1
            self.get_logger().warning(
                f"Compact sensor packet error: {exc}"
            )
            return False

    def process_sensor_data(self, message):
        """Parse legacy labeled sensor telemetry."""
        try:
            values = {}

            for part in message.split(","):
                if "=" not in part:
                    continue

                key, value = part.split("=", 1)
                values[key.strip()] = value.strip()

            required = (
                "TEMP",
                "HUMIDITY",
                "MQ4_ANALOG",
                "MQ4_DIGITAL",
                "LEFT_DISTANCE",
                "RIGHT_DISTANCE",
            )

            if not all(k in values for k in required):
                self.invalid_sensor_packets += 1
                return False

            temperature = float(values["TEMP"])
            humidity = float(values["HUMIDITY"])
            gas = int(float(values["MQ4_ANALOG"]))
            gas_status = int(float(values["MQ4_DIGITAL"]))
            left = float(values["LEFT_DISTANCE"])
            right = float(values["RIGHT_DISTANCE"])

            return self._publish_sensor_values(
                temperature,
                humidity,
                gas,
                gas_status,
                left,
                right,
            )

        except (ValueError, TypeError):
            self.invalid_sensor_packets += 1
            return False
        except Exception as exc:
            self.invalid_sensor_packets += 1
            self.get_logger().warning(
                f"Labeled sensor packet error: {exc}"
            )
            return False

    def print_sensor_terminal(self):
        with self.lock:
            t = self.temperature
            h = self.humidity
            gas = self.mq4_analog
            gd = self.mq4_digital
            left = self.left_distance
            right = self.right_distance
            count = self.sensor_packet_count
            last = self.last_sensor_time
            status = self.last_arduino_status

        age = time.monotonic() - last if last > 0 else float('inf')
        sensor_state = 'REALTIME' if age < 2.5 else 'STALE/WAITING'
        self.get_logger().info(
            f'SENSORS [{sensor_state}] | TEMP={t:.1f}C | HUM={h:.1f}% | '
            f'MQ4={gas} D={gd} | ULTRA L={left:.1f}cm R={right:.1f}cm | '
            f'packets={count} invalid={self.invalid_sensor_packets} | age={age:.2f}s | Arduino={status}'
        )

    # =============================================================
    # SPEED
    # =============================================================

    def set_speed_mode(self, mode):
        mode = int(mode) % 3
        with self.lock:
            self.speed_mode = mode
        msg = Int8()
        msg.data = mode
        try:
            self.speed_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f'Speed topic publish error: {exc}')
        self.send_speed_mode(mode, force=True)
        self.get_logger().info(f'SPEED MODE: {self.get_speed_mode_name(mode)}')

    def get_speed_multiplier(self, mode=None):
        if mode is None:
            mode = self.speed_mode
        return (self.SLOW_MULTIPLIER, self.NORMAL_MULTIPLIER, self.TURBO_MULTIPLIER)[int(mode)]

    def get_speed_mode_name(self, mode=None):
        if mode is None:
            mode = self.speed_mode
        return ('SLOW', 'NORMAL', 'TURBO')[int(mode)]

    # =============================================================
    # MOTOR MODEL — exact source mapping
    # =============================================================

    def calculate_motor_commands(self, x=None, y=None):
        with self.lock:
            if x is None:
                x = self.joy_x
            if y is None:
                y = self.joy_y

        m1 = m2 = m3 = m4 = 0.0

        # DO NOT change: this is the verified Arduino-compatible mapping.
        if y > 0.5:
            m1 = m2 = m3 = m4 = 1.0
        elif y < -0.5:
            m1 = m2 = m3 = m4 = -1.0
        elif x > 0.5:
            m1 = m2 = 1.0
            m3 = m4 = -1.0
        elif x < -0.5:
            m1 = m2 = -1.0
            m3 = m4 = 1.0

        return m1, m2, m3, m4

    def command_direction(self, x, y):
        # Preserve source priority: Y direction is selected before X.
        if y > 0.5:
            return (0, 1)
        if y < -0.5:
            return (0, -1)
        if x > 0.5:
            return (1, 0)
        if x < -0.5:
            return (-1, 0)
        return (0, 0)

    def _record_manual_command(self, x, y, now):
        direction = self.command_direction(x, y)
        mode = self.speed_mode
        self.recorder.update(direction, mode, now)

    # =============================================================
    # AUTONOMOUS POINT-TO-POINT NAVIGATION
    # =============================================================

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def set_autonomous_target(self, x, y):
        """Set a map target without moving the physical rover."""
        try:
            tx, ty = float(x), float(y)
            if not (math.isfinite(tx) and math.isfinite(ty)):
                raise ValueError('target must be finite')
        except Exception as exc:
            self.get_logger().warning(f'Invalid autonomous target: {exc}')
            return False

        with self.lock:
            self.autonomous_target = (tx, ty)
            self.autonomous_active = False
            self.autonomous_status = 'TARGET SELECTED — PRESS AUTO BUTTON'
            self.autonomous_distance_remaining = math.hypot(tx - self.x, ty - self.y)
            self.autonomous_heading_error = 0.0
        self.publish_autonomous_status()
        self.get_logger().info(f'AUTONOMOUS TARGET SET: X={tx:.2f} Y={ty:.2f}')
        return True

    def clear_autonomous_target(self):
        self.cancel_autonomous('TARGET CLEARED', send_stop=False)
        with self.lock:
            self.autonomous_target = None
            self.autonomous_distance_remaining = 0.0
            self.autonomous_heading_error = 0.0
            self.autonomous_status = 'MANUAL'

    def start_autonomous(self):
        now = time.monotonic()
        with self.lock:
            if self.rollback_active:
                self.get_logger().warning('Autonomous start ignored during rollback.')
                return False
            if self.autonomous_target is None:
                self.autonomous_status = 'NO TARGET — CLICK MAP FIRST'
                self.get_logger().warning('Autonomous start ignored: no map target selected.')
                return False
            distance = math.hypot(self.autonomous_target[0] - self.x,
                                  self.autonomous_target[1] - self.y)
            if distance <= self.AUTO_TARGET_TOLERANCE:
                self.autonomous_status = 'DESTINATION ALREADY REACHED'
                return False
            self.autonomous_active = True
            self.autonomous_status = 'AUTONOMOUS ACTIVE'
            self.last_command_time = now
        self.set_full_control(True)
        self.publish_autonomous_status()
        # Autonomous uses SLOW for stable demo navigation, then restores the
        # operator-selected mode when navigation finishes/cancels.
        self._set_autonomous_speed_mode(True)
        self.get_logger().info('AUTONOMOUS POINT-TO-POINT STARTED')
        return True

    def _set_autonomous_speed_mode(self, active):
        if active:
            with self.lock:
                if not hasattr(self, '_autonomous_saved_speed_mode'):
                    self._autonomous_saved_speed_mode = self.speed_mode
                mode = self.AUTO_SLOW_MODE
                self.speed_mode = mode
        else:
            with self.lock:
                if not hasattr(self, '_autonomous_saved_speed_mode'):
                    return
                mode = self._autonomous_saved_speed_mode
                del self._autonomous_saved_speed_mode
                self.speed_mode = mode
        self.send_speed_mode(mode, force=True)
        try:
            msg = Int8()
            msg.data = int(mode)
            self.speed_pub.publish(msg)
        except Exception:
            pass

    def cancel_autonomous(self, reason='AUTONOMOUS CANCELLED', send_stop=True):
        with self.lock:
            was_active = self.autonomous_active
            self.autonomous_active = False
            self.autonomous_status = str(reason)
            self.autonomous_distance_remaining = (
                math.hypot(self.autonomous_target[0] - self.x, self.autonomous_target[1] - self.y)
                if self.autonomous_target is not None else 0.0
            )
            self.autonomous_heading_error = 0.0
        self.publish_autonomous_status()
        if was_active:
            self.recorder.finish(time.monotonic())
            if send_stop:
                self.stop_rover()
            self._set_autonomous_speed_mode(False)
            if not self.rollback_active:
                self.set_full_control(False)
            self.get_logger().warning(str(reason))

    def _autonomous_step(self, now):
        """Closed-loop point navigation using only the existing cardinal commands."""
        with self.lock:
            if not self.autonomous_active or self.autonomous_target is None:
                return
            tx, ty = self.autonomous_target
            x, y, theta = self.x, self.y, self.theta

        dx, dy = tx - x, ty - y
        distance = math.hypot(dx, dy)
        if distance <= self.AUTO_TARGET_TOLERANCE:
            self._finish_autonomous()
            return

        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - theta)

        # Hysteresis prevents rapid LEFT/RIGHT oscillation around the target bearing.
        driving = abs(heading_error) <= self.AUTO_HEADING_REACQUIRE
        if not driving:
            cmd_x, cmd_y = (-1.0, 0.0) if heading_error > 0.0 else (1.0, 0.0)
            status = 'ALIGNING TO TARGET'
        else:
            cmd_x, cmd_y = 0.0, 1.0
            status = 'DRIVING TO TARGET'
            if distance <= self.AUTO_SLOW_DISTANCE:
                status = 'FINAL APPROACH'

        with self.lock:
            self.joy_x, self.joy_y = cmd_x, cmd_y
            self.last_command_time = now
            self.autonomous_distance_remaining = distance
            self.autonomous_heading_error = heading_error
            self.autonomous_status = status
            mode = self.speed_mode

        self.publish_autonomous_status()
        self.recorder.update(self.command_direction(cmd_x, cmd_y), mode, now)
        self._send_motion_if_needed(cmd_x, cmd_y, now, force=False)

    def _finish_autonomous(self):
        now = time.monotonic()
        self.recorder.finish(now)
        with self.lock:
            self.autonomous_active = False
            self.autonomous_status = 'DESTINATION REACHED'
            self.autonomous_distance_remaining = 0.0
            self.autonomous_heading_error = 0.0
            self.joy_x = self.joy_y = 0.0
            self.last_command_time = now
        self.publish_autonomous_status()
        self._send_motion_if_needed(0.0, 0.0, now, force=True)
        self._set_autonomous_speed_mode(False)
        self.set_full_control(False)
        self.get_logger().info('AUTONOMOUS DESTINATION REACHED (VIRTUAL/PREDICTED)')

    # =============================================================
    # ROLLBACK
    # =============================================================

    @staticmethod
    def inverse_direction(x, y):
        # Exact inverse mapping required by the prototype behavior.
        return -int(x), -int(y)

    def start_rollback(self):
        now = time.monotonic()
        self.cancel_autonomous('ROLLBACK REQUESTED')
        self.recorder.finish(now)
        segments = self.recorder.snapshot()
        if not segments:
            self.get_logger().warning('Rollback unavailable: no exploration path recorded.')
            return False

        with self.lock:
            self.rollback_active = True
            self.rollback_status = 'AUTO ROLLBACK ACTIVE'
            self.rollback_progress = 0.0
            self.rollback_index = len(segments) - 1
            self.rollback_segment_elapsed = 0.0
            self.rollback_last_time = now
            self.rollback_distance_remaining = math.hypot(self.x, self.y)
            self.rollback_trail.clear()
            self.rollback_trail.append((self.x, self.y, self.theta))
            self.rollback_path_points = list(self.exploration_points)
            self.rollback_path_cursor = len(self.rollback_path_points) - 1
            self.rollback_final_correction = False
            self.joy_x = self.joy_y = 0.0
            self.last_command_time = now

        self._send_motion_if_needed(0.0, 0.0, now, force=True)
        self.set_full_control(True)
        self.get_logger().info(f'AUTO ROLLBACK STARTED — {len(segments)} segments')
        return True

    def _truncate_rollback_path_locked(self):
        """Trim the explored path behind the rover continuously during rollback.

        The nearest recorded point to the current predicted pose becomes the
        visible path endpoint. The current pose is appended so the line follows
        the rover smoothly instead of disappearing in large chunks.
        """
        points = self.rollback_path_points
        if not points:
            return

        max_cursor = min(self.rollback_path_cursor, len(points) - 1)
        if max_cursor < 0:
            return

        # Rollback only moves toward the beginning, so search a small backward
        # window instead of scanning the whole path every odometry tick.
        best_cursor = max_cursor
        best_distance = math.inf
        start = max(0, max_cursor - 12)

        for idx in range(start, max_cursor + 1):
            px, py = points[idx][0], points[idx][1]
            distance = math.hypot(self.x - px, self.y - py)
            if distance < best_distance:
                best_distance = distance
                best_cursor = idx

        self.rollback_path_cursor = best_cursor
        keep = list(points[:best_cursor + 1])

        # Make the visible endpoint exactly follow the live rover position.
        if not keep or math.hypot(
            keep[-1][0] - self.x, keep[-1][1] - self.y
        ) > 0.001:
            keep.append((self.x, self.y, self.theta))

        self.exploration_points.clear()
        self.exploration_points.extend(keep)

    def _rollback_step(self, now):
        with self.lock:
            if not self.rollback_active:
                return
            dt = max(0.0, min(now - self.rollback_last_time, 0.10))
            self.rollback_last_time = now
            x, y = self.x, self.y

            # Once the recorded segments have been replayed, use closed-loop
            # correction to eliminate drift instead of snapping the GUI to (0,0).
            if self.rollback_final_correction:
                distance = math.hypot(x, y)
                heading_error = self.normalize_angle(-self.theta)
                if distance <= 0.06 and abs(heading_error) <= math.radians(5.0):
                    complete = True
                    command = (0.0, 0.0)
                    rollback_mode = self.speed_mode
                elif distance > 0.06 and abs(heading_error) > math.radians(10.0):
                    complete = False
                    command = (-1.0, 0.0) if heading_error > 0 else (1.0, 0.0)
                    rollback_mode = 0
                elif distance > 0.06:
                    complete = False
                    target_heading = math.atan2(-y, -x)
                    err = self.normalize_angle(target_heading - self.theta)
                    command = (-1.0, 0.0) if err > 0 else (1.0, 0.0) if abs(err) > math.radians(8) else (0.0, 1.0)
                    rollback_mode = 0
                else:
                    complete = False
                    command = (-1.0, 0.0) if heading_error > 0 else (1.0, 0.0)
                    rollback_mode = 0
                self.joy_x, self.joy_y = command
                self.speed_mode = rollback_mode
                self.rollback_distance_remaining = distance
                self.rollback_status = 'FINAL RETURN TO START'
                self.rollback_progress = 99.0
            else:
                segments = self.recorder.snapshot()
                if not segments or self.rollback_index < 0:
                    self.rollback_final_correction = True
                    command = (0.0, 0.0)
                    rollback_mode = 0
                    complete = False
                else:
                    seg = segments[self.rollback_index]
                    duration = max(0.0, float(seg.get('duration', 0.0)))
                    self.rollback_segment_elapsed += dt
                    command = tuple(float(v) for v in self.inverse_direction(seg['x'], seg['y']))
                    rollback_mode = int(seg['mode'])
                    self.speed_mode = rollback_mode
                    self.joy_x, self.joy_y = command
                    complete = False
                    if self.rollback_segment_elapsed >= duration:
                        self.rollback_index -= 1
                        self.rollback_segment_elapsed = 0.0
                        if self.rollback_index < 0:
                            self.rollback_final_correction = True
                    total = max(1, len(segments))
                    done = total - max(0, self.rollback_index + 1)
                    self.rollback_progress = min(99.0, done * 100.0 / total)
                    self.rollback_distance_remaining = math.hypot(x, y)
                    self.rollback_status = 'AUTO ROLLBACK ACTIVE'

            if not complete:
                self.last_command_time = now

        if complete:
            self.complete_rollback()
            return

        self.send_speed_mode(rollback_mode, force=False)
        self._send_motion_if_needed(command[0], command[1], now, force=False)

    def complete_rollback(self):
        now = time.monotonic()
        self._send_motion_if_needed(0.0, 0.0, now, force=True)
        self.recorder.clear()
        with self.lock:
            self.rollback_active = False
            self.rollback_index = -1
            self.rollback_segment_elapsed = 0.0
            self.rollback_progress = 100.0
            self.rollback_distance_remaining = 0.0
            self.rollback_status = 'START REACHED — NEW EXPLORATION'
            self.joy_x = self.joy_y = 0.0
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.m1_rpm = self.m2_rpm = self.m3_rpm = self.m4_rpm = 0.0
            self.rollback_trail.clear()
            self.exploration_points.clear()
            self.exploration_points.append((0.0, 0.0, 0.0))
            self.rollback_path_points = []
            self.rollback_path_cursor = -1
            self.rollback_final_correction = False
            self.autonomous_target = None
            self.autonomous_active = False
            self.autonomous_status = 'MANUAL'
            self.autonomous_distance_remaining = 0.0
            self.autonomous_heading_error = 0.0
            self.last_command_time = now
            self.last_odom_time = now
        self.set_full_control(False)
        self.get_logger().info('ROLLBACK COMPLETE — PATH CLEARED, NEW EXPLORATION STARTED')

    def cancel_rollback(self):
        with self.lock:
            if not self.rollback_active:
                return
            self.rollback_active = False
            self.rollback_status = 'ROLLBACK CANCELLED — RETURNED PATH REMOVED'
            self.rollback_index = -1
            self.rollback_segment_elapsed = 0.0
            # Keep exactly the exploration prefix that has not yet been rolled back.
            if self.rollback_path_points:
                cursor = max(0, min(self.rollback_path_cursor, len(self.rollback_path_points)-1))
                self.exploration_points.clear()
                self.exploration_points.extend(self.rollback_path_points[:cursor+1])
            self.rollback_trail.clear()
            self.rollback_path_points = []
            self.rollback_path_cursor = -1
            self.rollback_final_correction = False
        self.stop_rover()
        self.set_full_control(False)
        self.get_logger().warning('AUTO ROLLBACK CANCELLED — RETURNED PATH REMOVED')

    def publish_rollback_status(self):
        try:
            with self.lock:
                status = self.rollback_status
                progress = self.rollback_progress
                remaining = self.rollback_distance_remaining
                mode = self.speed_mode
            msg = String()
            msg.data = (
                f'{status}|progress={progress:.1f}|'
                f'remaining={remaining:.2f}|mode={self.get_speed_mode_name(mode)}'
            )
            self.rollback_status_pub.publish(msg)
            self.publish_rover_telemetry()
        except Exception as exc:
            self.get_logger().warning(f'Rollback status publish error: {exc}')

    def publish_rover_telemetry(self):
        with self.lock:
            speed_mode = int(self.speed_mode)
            left_rpm = int(round((self.m1_rpm + self.m2_rpm) / 2.0))
            right_rpm = int(round((self.m3_rpm + self.m4_rpm) / 2.0))

        speed_msg = Int8()
        speed_msg.data = speed_mode
        self.speed_pub.publish(speed_msg)

        left_msg = Int32()
        left_msg.data = left_rpm
        self.motor_left_pub.publish(left_msg)

        right_msg = Int32()
        right_msg.data = right_rpm
        self.motor_right_pub.publish(right_msg)

    # =============================================================
    # VIRTUAL ODOMETRY
    # =============================================================

    def odometry_update(self):
        try:
            now = time.monotonic()
            with self.lock:
                dt = now - self.last_odom_time if hasattr(self, 'last_odom_time') else 1.0 / self.ODOMETRY_HZ
                self.last_odom_time = now

                if dt <= 0.0 or dt > 0.25:
                    return

                if not self.rollback_active:
                    command_age = now - self.last_command_time
                    if command_age > self.COMMAND_TIMEOUT:
                        x_cmd, y_cmd = 0.0, 0.0
                    else:
                        x_cmd, y_cmd = self.joy_x, self.joy_y
                else:
                    x_cmd, y_cmd = self.joy_x, self.joy_y

                m1, m2, m3, m4 = self.calculate_motor_commands(x_cmd, y_cmd)
                multiplier = self.get_speed_multiplier(self.speed_mode)
                left_velocity = m1 * self.MAX_SPEED * multiplier
                right_velocity = m3 * self.MAX_SPEED * multiplier

                circumference = math.pi * self.WHEEL_DIAMETER
                self.m1_rpm = left_velocity / circumference * 60.0
                self.m2_rpm = left_velocity / circumference * 60.0
                self.m3_rpm = right_velocity / circumference * 60.0
                self.m4_rpm = right_velocity / circumference * 60.0

                self.m1_rotations += self.m1_rpm / 60.0 * dt
                self.m2_rotations += self.m2_rpm / 60.0 * dt
                self.m3_rotations += self.m3_rpm / 60.0 * dt
                self.m4_rotations += self.m4_rpm / 60.0 * dt

                linear_velocity = (left_velocity + right_velocity) / 2.0
                angular_velocity = (right_velocity - left_velocity) / self.WHEEL_BASE
                distance_delta = abs(linear_velocity * dt)

                self.total_distance += distance_delta
                heading_delta = angular_velocity * dt
                if abs(angular_velocity) < 1.0e-9:
                    self.x += linear_velocity * math.cos(self.theta) * dt
                    self.y += linear_velocity * math.sin(self.theta) * dt
                else:
                    radius = linear_velocity / angular_velocity
                    next_theta = self.theta + heading_delta
                    self.x += radius * (math.sin(next_theta) - math.sin(self.theta))
                    self.y += radius * (-math.cos(next_theta) + math.cos(self.theta))
                self.theta += heading_delta
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
                self.linear_velocity = linear_velocity
                self.angular_velocity = angular_velocity

                if self.rollback_active:
                    self.rollback_trail.append((self.x, self.y, self.theta))
                    self._truncate_rollback_path_locked()
                elif distance_delta > 0.0001:
                    self.exploration_points.append((self.x, self.y, self.theta))

                # Copy only small scalar state while holding lock.
                x, y, theta = self.x, self.y, self.theta
                lv, av = linear_velocity, angular_velocity
                rotations = (
                    self.m1_rotations,
                    self.m2_rotations,
                    self.m3_rotations,
                    self.m4_rotations,
                )

            self.publish_odom(now, x, y, theta, lv, av)
        except Exception as exc:
            self.get_logger().error(f'Odometry callback error: {exc}')
            self.stop_rover()

    def publish_odom(self, now, x, y, theta, linear_velocity, angular_velocity):
        try:
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.z = math.sin(theta / 2.0)
            msg.pose.pose.orientation.w = math.cos(theta / 2.0)
            msg.twist.twist.linear.x = linear_velocity
            msg.twist.twist.angular.z = angular_velocity
            self.odom_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f'Odometry publish error: {exc}')

    def publish_path(self):
        """Publish the bounded live path at 50 Hz for near-realtime map updates."""
        try:
            with self.lock:
                points = list(self.exploration_points)
                now_msg = self.get_clock().now().to_msg()

            path = Path()
            path.header.stamp = now_msg
            path.header.frame_id = 'odom'
            for px, py, ptheta in points:
                pose = PoseStamped()
                pose.header.stamp = now_msg
                pose.header.frame_id = 'odom'
                pose.pose.position.x = px
                pose.pose.position.y = py
                pose.pose.orientation.z = math.sin(ptheta / 2.0)
                pose.pose.orientation.w = math.cos(ptheta / 2.0)
                path.poses.append(pose)
            self.path_pub.publish(path)
        except Exception as exc:
            self.get_logger().warning(f'Path publish error: {exc}')

    # =============================================================
    # RESET
    # =============================================================

    def reset_callback(self, request, response):
        try:
            self.cancel_rollback()
            self.stop_rover()
            with self.lock:
                self.x = 0.0
                self.y = 0.0
                self.theta = 0.0
                self.m1_rotations = 0.0
                self.m2_rotations = 0.0
                self.m3_rotations = 0.0
                self.m4_rotations = 0.0
                self.m1_rpm = self.m2_rpm = self.m3_rpm = self.m4_rpm = 0.0
                self.linear_velocity = 0.0
                self.angular_velocity = 0.0
                self.total_distance = 0.0
                self.joy_x = self.joy_y = 0.0
                self.exploration_points.clear()
                self.exploration_points.append((0.0, 0.0, 0.0))
                self.rollback_trail.clear()
                self.rollback_progress = 0.0
                self.rollback_distance_remaining = 0.0
                self.rollback_status = 'EXPLORATION'
                self.autonomous_active = False
                self.autonomous_status = 'MANUAL'
                self.autonomous_target = None
                self.autonomous_distance_remaining = 0.0
                self.autonomous_heading_error = 0.0
                self.last_odom_time = time.monotonic()
            self.recorder.clear()
            self.last_sent_x = None
            self.last_sent_y = None

            # Arduino sketch will be updated next to interpret RESET\\n.
            hardware_reset_sent = self.bt.send('RESET\\n')

            self.send_speed_mode(self.speed_mode, force=True)
            self.stop_rover()
            response.success = True
            if hardware_reset_sent:
                response.message = 'Virtual rover reset + Arduino RESET command sent'
            else:
                response.message = 'Virtual rover reset; Arduino RESET command could not be sent'
            self.get_logger().info('Virtual rover RESET')
        except Exception as exc:
            response.success = False
            response.message = f'Reset failed: {exc}'
            self.get_logger().error(f'Reset error: {exc}')
        return response

    # =============================================================
    # MAP UI — persistent artists, no ax.clear(), no full rebuild
    # =============================================================

    def setup_map_ui(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(13, 9))
        try:
            self.fig.canvas.manager.set_window_title('DrillPulse — Rover Control & Live Map V2')
        except Exception:
            pass

        self.ax.set_title('DRILLPULSE — LIVE ROVER MAP V2')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.grid(True, linestyle='--', alpha=0.45)

        self.start_marker, = self.ax.plot(
            [0], [0], marker='o', markersize=10, linestyle='None', label='START'
        )
        self.ax.text(0.15, 0.15, 'START', fontsize=9)

        self.trajectory_line, = self.ax.plot(
            [], [], linewidth=2, label='Predicted trajectory'
        )
        self.rollback_line, = self.ax.plot(
            [], [], linewidth=2, linestyle='--', label='Rollback trail'
        )

        self.rover = Polygon(
            self.get_rover_shape(0, 0, 0), closed=True, linewidth=2
        )
        self.ax.add_patch(self.rover)

        self.heading_line, = self.ax.plot([], [], linewidth=2)
        self.position_marker, = self.ax.plot(
            [], [], marker='o', markersize=5, linestyle='None'
        )
        self.target_marker, = self.ax.plot(
            [], [], marker='x', markersize=12, markeredgewidth=2,
            linestyle='None', label='Autonomous target'
        )
        self.target_line, = self.ax.plot(
            [], [], linestyle=':', linewidth=1.5, label='Target bearing'
        )

        self.info_text = self.ax.text(
            0.015, 0.985, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=9, family='monospace',
            bbox=dict(boxstyle='round', alpha=0.85)
        )
        self.rollback_text = self.ax.text(
            0.985, 0.985, '', transform=self.ax.transAxes,
            verticalalignment='top', horizontalalignment='right',
            fontsize=9, family='monospace', bbox=dict(boxstyle='round', alpha=0.85)
        )
        self.motor_text = self.ax.text(
            0.985, 0.32, '', transform=self.ax.transAxes,
            verticalalignment='top', horizontalalignment='right',
            fontsize=8.5, family='monospace', bbox=dict(boxstyle='round', alpha=0.80)
        )
        self.help_text = self.ax.text(
            0.015, 0.015,
            'BUTTON 0: SPEED | BUTTON 2: ROLLBACK | '
            'MAP CLICK: TARGET | AUTO BUTTON: START/CANCEL | '
            'A: AUTO | RIGHT CLICK: CLEAR TARGET | R: RESET | F: FOLLOW | '
            'G: GRID | H: HEADING | S: SAVE | Q: QUIT',
            transform=self.ax.transAxes, fontsize=8.5
        )

        # On-screen reset requests both a virtual-map reset and an Arduino reset.
        reset_ax = self.fig.add_axes([0.875, 0.025, 0.095, 0.045])
        self.reset_button = Button(reset_ax, 'RESET')
        self.reset_button.on_clicked(self._reset_button_callback)

        self.fig.canvas.mpl_connect('key_press_event', self.keyboard_callback)
        self.fig.canvas.mpl_connect('button_press_event', self.map_mouse_callback)
        self._last_gui_xlim = self.ax.get_xlim()
        self._last_gui_ylim = self.ax.get_ylim()

    def get_rover_shape(self, x, y, theta):
        c = math.cos(theta)
        s = math.sin(theta)
        result = []
        for px, py in ((0.30, 0.0), (-0.20, 0.18), (-0.20, -0.18)):
            result.append([x + px * c - py * s, y + px * s + py * c])
        return result

    def _gui_snapshot(self):
        # Copy bounded visualization data under a short lock, then render outside it.
        with self.lock:
            return {
                'x': self.x,
                'y': self.y,
                'theta': self.theta,
                'lv': self.linear_velocity,
                'av': self.angular_velocity,
                'speed_mode': self.speed_mode,
                'rpm': (self.m1_rpm, self.m2_rpm, self.m3_rpm, self.m4_rpm),
                'rot': (
                    self.m1_rotations,
                    self.m2_rotations,
                    self.m3_rotations,
                    self.m4_rotations,
                ),
                'distance': self.total_distance,
                'trajectory': list(self.exploration_points),
                'rollback_trail': list(self.rollback_trail),
                'rollback_active': self.rollback_active,
                'rollback_status': self.rollback_status,
                'progress': self.rollback_progress,
                'remaining': self.rollback_distance_remaining,
                'rollback_index': self.rollback_index,
                'segments': self.recorder.count(),
                'connected': self.bt.connected,
                'controller': self.controller_connected,
                'autonomous_active': self.autonomous_active,
                'autonomous_status': self.autonomous_status,
                'autonomous_target': self.autonomous_target,
                'autonomous_remaining': self.autonomous_distance_remaining,
                'autonomous_heading_error': self.autonomous_heading_error,
                'autonomous_button': self.AUTONOMOUS_BUTTON,
            }

    def update_map(self):
        if not self.gui_alive or not plt.fignum_exists(self.fig.number):
            return False

        try:
            snap = self._gui_snapshot()
            x = snap['x']
            y = snap['y']
            theta = snap['theta']

            trajectory = snap['trajectory']
            tx = [p[0] for p in trajectory]
            ty = [p[1] for p in trajectory]
            rb = snap['rollback_trail']
            rx = [p[0] for p in rb]
            ry = [p[1] for p in rb]

            self.trajectory_line.set_data(tx, ty)
            self.rollback_line.set_data(rx, ry)
            self.rover.set_xy(self.get_rover_shape(x, y, theta))
            self.position_marker.set_data([x], [y])

            target = snap['autonomous_target']
            if target is not None:
                self.target_marker.set_data([target[0]], [target[1]])
                self.target_line.set_data([x, target[0]], [y, target[1]])
            else:
                self.target_marker.set_data([], [])
                self.target_line.set_data([], [])

            if self.show_heading:
                h = 0.50
                self.heading_line.set_data(
                    [x, x + h * math.cos(theta)],
                    [y, y + h * math.sin(theta)],
                )
            else:
                self.heading_line.set_data([], [])

            heading_deg = math.degrees(theta) % 360.0
            mode_name = self.get_speed_mode_name(snap['speed_mode'])
            status = 'CONNECTED' if snap['connected'] else 'BT DISCONNECTED'
            controller = 'CONNECTED' if snap['controller'] else 'DISCONNECTED'

            info = (
                'DRILLPULSE ROVER V2\n'
                '────────────────────────\n'
                f'Position X     : {x:7.2f} m\n'
                f'Position Y     : {y:7.2f} m\n'
                f'Heading        : {heading_deg:7.1f} deg\n'
                f'Linear speed   : {snap["lv"]:7.2f} m/s\n'
                f'Angular speed  : {snap["av"]:7.2f} rad/s\n'
                f'Speed mode     : {mode_name}\n'
                f'Total distance : {snap["distance"]:7.2f} m\n'
                f'Path points    : {len(tx):7d}\n'
                f'Bluetooth      : {status}\n'
                f'Joystick       : {controller}\n'
                '────────────────────────\n'
                f'Auto status    : {snap["autonomous_status"]}\n'
                f'Auto remaining : {snap["autonomous_remaining"]:7.2f} m\n'
                f'Auto heading   : {math.degrees(snap["autonomous_heading_error"]):7.1f} deg\n'
                'ODOMETRY: VIRTUAL / PREDICTED'
            )
            self.info_text.set_text(info)

            if snap['rollback_active']:
                segment_no = max(0, snap['rollback_index'] + 1)
                rollback_panel = (
                    'AUTO ROLLBACK\n'
                    '────────────────────\n'
                    'STATUS     : ACTIVE\n'
                    f'PROGRESS   : {snap["progress"]:6.1f} %\n'
                    f'REMAINING  : {snap["remaining"]:6.2f} m\n'
                    f'SEGMENT    : {segment_no}/{snap["segments"]}\n'
                    f'MODE       : {mode_name}\n'
                    'BUTTON 2   : CANCEL'
                )
            else:
                auto_button_text = (
                    f'BUTTON {snap["autonomous_button"]}'
                    if snap['autonomous_button'] is not None else 'KEY A'
                )
                rollback_panel = (
                    'NAVIGATION MODE\n'
                    '────────────────────\n'
                    f'ROLLBACK   : {snap["rollback_status"]}\n'
                    f'AUTONOMOUS : {snap["autonomous_status"]}\n'
                    f'TARGET REM : {snap["autonomous_remaining"]:6.2f} m\n'
                    f'RECORDED   : {snap["segments"]} segments\n'
                    'ROLLBACK   : BUTTON 2\n'
                    f'AUTO       : {auto_button_text}'
                )
            self.rollback_text.set_text(rollback_panel)

            rpm = snap['rpm']
            rot = snap['rot']
            motor_panel = (
                'MOTOR TELEMETRY\n'
                '────────────────────\n'
                f'M1 RPM : {rpm[0]:8.1f}   Rot: {rot[0]:8.2f}\n'
                f'M2 RPM : {rpm[1]:8.1f}   Rot: {rot[1]:8.2f}\n'
                f'M3 RPM : {rpm[2]:8.1f}   Rot: {rot[2]:8.2f}\n'
                f'M4 RPM : {rpm[3]:8.1f}   Rot: {rot[3]:8.2f}\n'
                '────────────────────\n'
                f'MAX PWM : {self.MAX_PWM}\n'
                f'MAX SPEED: {self.MAX_SPEED:.2f} m/s'
            )
            self.motor_text.set_text(motor_panel)

            # Stable follow mode: only move bounds when rover leaves a margin.
            if self.follow_rover:
                xmin, xmax = self.ax.get_xlim()
                ymin, ymax = self.ax.get_ylim()
                margin_x = (xmax - xmin) * 0.20
                margin_y = (ymax - ymin) * 0.20
                changed = False
                if x < xmin + margin_x or x > xmax - margin_x:
                    self.ax.set_xlim(x - 5.0, x + 5.0)
                    changed = True
                if y < ymin + margin_y or y > ymax - margin_y:
                    self.ax.set_ylim(y - 5.0, y + 5.0)
                    changed = True
                if changed:
                    self._last_gui_xlim = self.ax.get_xlim()
                    self._last_gui_ylim = self.ax.get_ylim()

            # draw_idle/flush are intentionally only here, never in control/odom callbacks.
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            return True
        except Exception as exc:
            self.gui_alive = False
            self.get_logger().error(f'Map rendering failure: {exc}')
            # GUI failure must not leave the rover moving.
            self.stop_rover()
            return False

    # =============================================================
    # KEYBOARD / GUI
    # =============================================================

    def map_mouse_callback(self, event):
        try:
            if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
                return
            if event.button == 1:
                self.set_autonomous_target(event.xdata, event.ydata)
            elif event.button == 3:
                self.clear_autonomous_target()
        except Exception as exc:
            self.get_logger().error(f'Map mouse error: {exc}')
            self.stop_rover()

    def _reset_button_callback(self, event):
        """Reset virtual odometry/map and request an Arduino-side reset."""
        try:
            self.reset_map()
        except Exception as exc:
            self.get_logger().error(f'GUI reset button error: {exc}')
            self.stop_rover()

    def keyboard_callback(self, event):
        try:
            if event.key == 'r':
                self.reset_map()
            elif event.key == 'f':
                self.follow_rover = not self.follow_rover
            elif event.key == 'g':
                self.show_grid = not self.show_grid
                self.ax.grid(self.show_grid)
            elif event.key == 'h':
                self.show_heading = not self.show_heading
            elif event.key == 's':
                filename = 'drillpulse_rollback_map.png'
                self.fig.savefig(filename, dpi=160, bbox_inches='tight')
                self.get_logger().info(f'Map saved: {filename}')
            elif event.key == 'a':
                if self.autonomous_active:
                    self.cancel_autonomous('AUTONOMOUS CANCELLED BY KEY A', send_stop=False)
                else:
                    self.start_autonomous()
            elif event.key == 'q':
                self.gui_alive = False
                self.stop_rover()
                plt.close(self.fig)
        except Exception as exc:
            self.get_logger().error(f'Keyboard/GUI error: {exc}')
            self.stop_rover()

    def reset_map(self):
        # Keep the source behavior: reset internally instead of waiting on ROS service.
        self.reset_callback(Trigger.Request(), Trigger.Response())

    # =============================================================
    # SHUTDOWN
    # =============================================================

    def shutdown(self):
        self.shutdown_event.set()
        try:
            self.cancel_autonomous('SHUTDOWN')
            self.stop_rover()
        except Exception:
            pass

        if self.control_thread.is_alive():
            self.control_thread.join(timeout=1.5)

        try:
            self.bt.safe_stop()
        except Exception:
            pass
        try:
            self.bt.close()
        except Exception:
            pass
        try:
            pygame.quit()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None

    try:
        node = DrillPulseRover()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)

        # ROS callbacks run away from the Matplotlib main thread.
        executor_thread = threading.Thread(
            target=executor.spin,
            name='drillpulse-ros',
            daemon=True,
        )
        executor_thread.start()

        # Matplotlib GUI stays on the main thread, where GUI backends are safest.
        gui_period = 1.0 / node.GUI_HZ
        next_gui = time.monotonic()
        while rclpy.ok() and node.gui_alive and plt.fignum_exists(node.fig.number):
            now = time.monotonic()
            if now >= next_gui:
                if not node.update_map():
                    break
                next_gui = now + gui_period
            # Do not use plt.pause() as the control clock. It is only GUI event
            # processing; the actual rover control loop is independent.
            plt.pause(0.001)

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'DrillPulse error: {exc}')
        if node is not None:
            try:
                node.stop_rover()
            except Exception:
                pass
    finally:
        if node is not None:
            try:
                node.shutdown()
            except Exception:
                pass

        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                try:
                    executor.shutdown()
                except Exception:
                    pass

        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        try:
            plt.close('all')
        except Exception:
            pass
        try:
            pygame.quit()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
