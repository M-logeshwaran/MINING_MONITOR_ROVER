# rover_dashboard

A ROS2 node that serves a web-based **liquid glass** dashboard for a mine
rover. It subscribes to your rover's telemetry topics and streams every
update straight to the browser over a websocket — no page refresh, no
polling.

## Topics it subscribes to (defaults)

| Topic              | Type                              | Shown as               |
|---------------------|-----------------------------------|-------------------------|
| `live_feed`          | `sensor_msgs/msg/CompressedImage` | Live camera panel       |
| `thermal_feed`       | `sensor_msgs/msg/CompressedImage` | Thermal camera panel    |
| `temperature`        | `std_msgs/msg/Float32`            | Radial gauge (°C)       |
| `humidity`           | `std_msgs/msg/Float32`            | Radial gauge (%RH)      |
| `gas_reading`        | `std_msgs/msg/Float32`            | Radial gauge (ppm)      |
| `joystick_values`    | `sensor_msgs/msg/Joy`             | Live stick position     |

If your real rover publishes different types (e.g. raw `sensor_msgs/Image`
instead of `CompressedImage`, or a custom gas-sensor message), edit the
subscriptions in `rover_dashboard/dashboard_node.py` — everything else
(web server, UI) is decoupled from the exact message type.

Topic **names** don't require touching the code at all — every name is a
ROS2 parameter (see below), or just use `ros2 launch` remaps.

## 1. Install dependencies

```bash
pip install flask flask-socketio python-socketio --break-system-packages
```

(`rclpy`, `sensor_msgs`, `std_msgs` come from your ROS2 install.)

## 2. Build

Copy the `rover_dashboard` folder into your workspace's `src/`, then:

```bash
cd ~/your_ws
colcon build --packages-select rover_dashboard --symlink-install
source install/setup.bash
```

## 3. Run

```bash
ros2 launch rover_dashboard dashboard.launch.py web_port:=8080
```

or directly:

```bash
ros2 run rover_dashboard dashboard_node --ros-args -p web_port:=8080
```

Then, from any laptop/phone on the same network as the rover:

```
http://<rover-ip>:8080/
```

## 4. Point it at your real topic names

No code edits needed — pass parameters at launch:

```bash
ros2 launch rover_dashboard dashboard.launch.py \
  live_feed_topic:=/camera/front/compressed \
  thermal_feed_topic:=/camera/thermal/compressed \
  temperature_topic:=/sensors/temperature \
  humidity_topic:=/sensors/humidity \
  gas_reading_topic:=/sensors/gas \
  joystick_topic:=/joy
```

## 5. Customizing the dashboard (in the browser, no code)

Click the gear icon top-right to open **Customize Dashboard**:

- **Drag** any panel by its title bar to reorder it.
- **Toggle** panels on/off if you don't need all six right now.
- **Layout density**: Compact / Comfortable / Spacious spacing.
- **Alert thresholds**: set the gas (ppm) and temperature (°C) levels that
  turn a badge amber/red.
- **Reset layout** puts everything back to default.

All of this is saved in the browser (`localStorage`), per device, so a
phone and a control-room monitor can each keep their own layout.

The grid itself is fully responsive: on a wide control-room monitor the
two camera feeds sit side-by-side at double width; on a phone everything
stacks into a single column automatically — no separate "mobile mode" to
configure.

## 6. Two-way control (optional)

The browser already opens a socket connection and can emit a
`joystick_cmd` event (see `app.js` / `dashboard_node.py`'s
`on_joystick_cmd` handler) if you want to drive the rover from the
dashboard itself — wire that handler up to a `geometry_msgs/Twist` or
`sensor_msgs/Joy` publisher to close the loop.

## File map

```
rover_dashboard/
├── package.xml
├── setup.py / setup.cfg / MANIFEST.in
├── launch/dashboard.launch.py
├── resource/rover_dashboard
└── rover_dashboard/
    ├── dashboard_node.py      # ROS2 node + embedded Flask-SocketIO server
    └── web/
        ├── index.html         # panel layout
        ├── style.css          # liquid-glass theme, responsive grid
        └── app.js             # socket handling, gauges, joystick, settings
```
