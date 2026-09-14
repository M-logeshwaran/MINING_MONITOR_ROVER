#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  echo "[setup] Sourcing ROS: $ROS_SETUP"
  source "$ROS_SETUP"
else
  echo "[setup] ROS not found at $ROS_SETUP" >&2
  exit 1
fi

if [ ! -d ".venv" ]; then
  echo "[setup] Creating project virtual environment"
  python3 -m venv .venv
fi

source .venv/bin/activate

echo "[setup] Upgrading pip tooling"
python -m pip install --upgrade pip setuptools wheel

echo "[setup] Installing Python dependencies used by the rover dashboard"
python -m pip install \
  flask \
  flask-socketio \
  python-socketio \
  opencv-python \
  ultralytics \
  lap \
  numpy \
  pyyaml \
  colcon-common-extensions

echo "[setup] Building rover packages"
colcon build --packages-select rover_dashboard drillpulse_control --event-handlers console_direct+

source install/setup.bash

# Force the generated ROS entry points to use the project venv Python.
# This avoids the system /usr/bin/python3 shebang issue that caused ModuleNotFoundError.
for script in \
  install/rover_dashboard/lib/rover_dashboard/dashboard \
  install/rover_dashboard/lib/rover_dashboard/thermal
 do
  if [ -f "$script" ]; then
    sed -i '1s|^#!/usr/bin/python3$|#!/home/loki/ros2_ws/.venv/bin/python|' "$script"
    echo "[setup] Patched shebang for $script"
  fi
done

echo "[setup] Environment ready"

echo ""
echo "Use:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source /home/loki/ros2_ws/.venv/bin/activate"
echo "  source /home/loki/ros2_ws/install/setup.bash"
echo "  ros2 launch rover_dashboard dashboard.launch.py"
echo ""

if [ "${1:-}" = "launch" ]; then
  echo "[setup] Launching dashboard..."
  ros2 launch rover_dashboard dashboard.launch.py
fi

