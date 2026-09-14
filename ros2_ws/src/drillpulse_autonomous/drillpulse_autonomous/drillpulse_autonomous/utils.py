import math
import sys
import time

# ANSI Color Codes for terminal logging
COLOR_RESET = "\033[0m"
COLOR_BOLD = "\033[1m"
COLOR_RED = "\033[91m"
COLOR_GREEN = "\033[92m"
COLOR_YELLOW = "\033[93m"
COLOR_BLUE = "\033[94m"
COLOR_MAGENTA = "\033[95m"
COLOR_CYAN = "\033[96m"
COLOR_WHITE = "\033[97m"

TAG_COLORS = {
    "AUTO": COLOR_CYAN,
    "VISION": COLOR_MAGENTA,
    "SAFETY": COLOR_RED,
    "PLAN": COLOR_YELLOW,
    "MOTOR": COLOR_GREEN,
    "CAMERA": COLOR_BLUE,
    "ULTRASONIC": COLOR_YELLOW,
    "NAV": COLOR_GREEN,
}

def log_colored(tag: str, message: str, logger=None):
    """
    Prints a formatted, colorized log message to terminal and optionally ROS logger.
    Tags: [AUTO], [VISION], [SAFETY], [PLAN], [MOTOR], [CAMERA], [ULTRASONIC], [NAV]
    """
    color = TAG_COLORS.get(tag.upper(), COLOR_WHITE)
    formatted_msg = f"{color}{COLOR_BOLD}[{tag.upper()}]{COLOR_RESET} {message}"
    
    if logger is not None:
        if tag.upper() == "SAFETY" and "STOP" in message.upper():
            logger.warn(formatted_msg)
        else:
            logger.info(formatted_msg)
    else:
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] {formatted_msg}", flush=True)

def normalize_angle(angle_rad: float) -> float:
    """Normalizes an angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

def euclidean_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculates 2D Euclidean distance between two points."""
    return math.hypot(x2 - x1, y2 - y1)

def clamp(val: float, min_val: float, max_val: float) -> float:
    """Clamps a numeric value between min_val and max_val."""
    return max(min_val, min(val, max_val))
