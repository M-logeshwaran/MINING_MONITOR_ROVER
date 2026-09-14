#!/usr/bin/env python3
"""Fast ROS 2 thermal pipeline for compressed live camera frames."""

import threading

import cv2
import numpy as np
import torch

# ============================================================
# LIVE SYNTHETIC THERMAL CAMERA
# ============================================================
# Visual target: the supplied thermal-camera reference image.
#
# IMPORTANT:
# - The reference image is NOT used at runtime.
# - Your LIVE RGB webcam is transformed frame-by-frame.
# - Background stays cool blue/cyan.
# - PERSON / LAPTOP / MOBILE become hot thermal objects.
# - Other supported electronic objects receive heat only.
# - Only PERSON/LAPTOP/MOBILE receive labels + dashed boxes.
#
# This is SYNTHETIC thermal visualization. A normal RGB webcam
# cannot physically measure °C.
# ============================================================

MODEL_NAME = "yolo11n-seg.pt"
CAMERA_INDEX = 0

CONFIDENCE = 0.30
IOU = 0.50

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

WINDOW_NAME = "THERMAL VISION"

# Use CUDA automatically when available.
DEVICE = 0 if torch.cuda.is_available() else "cpu"

# ============================================================
# TEMPERATURE RANGES
# ============================================================

TEMP_RANGES = {
    "PERSON": (35.8, 38.2),
    "LAPTOP": (40.0, 50.0),
    "MOBILE": (34.0, 44.0),

    # Thermal-only electronics
    "TV": (30.0, 40.0),
    "MOUSE": (28.0, 35.0),
    "REMOTE": (27.0, 34.0),
    "KEYBOARD": (29.0, 38.0),
}

BACKGROUND_TEMP = (20.0, 29.0)

# ============================================================
# COCO CLASS IDs
# ============================================================

PERSON_ID = 0
TV_ID = 62
LAPTOP_ID = 63
MOUSE_ID = 64
REMOTE_ID = 65
KEYBOARD_ID = 66
MOBILE_ID = 67

LABELED = {
    PERSON_ID: "PERSON",
    LAPTOP_ID: "LAPTOP",
    MOBILE_ID: "MOBILE",
}

THERMAL_ONLY = {
    TV_ID: "TV",
    MOUSE_ID: "MOUSE",
    REMOTE_ID: "REMOTE",
    KEYBOARD_ID: "KEYBOARD",
}


# ============================================================
# REFERENCE-STYLE THERMAL LUT
# Colors sampled/approximated from the supplied reference image.
# OpenCV BGR order.
# ============================================================

def make_reference_lut():
    # Deep blue -> electric blue -> cyan -> green -> yellow
    # -> orange -> red -> white.
    rgb = np.array([
        [0,   18,  72],
        [0,   25, 176],
        [0,   44, 216],
        [0,   81, 248],
        [2,  150, 249],
        [8,  220, 191],
        [18, 245,  67],
        [130,249,  6],
        [237,236,  4],
        [252,127,  7],
        [250, 26,  1],
        [210,  0,  0],
        [255, 70, 70],
        [255,255,255],
    ], dtype=np.uint8)

    # Convert RGB palette to BGR for OpenCV.
    bgr = rgb[:, ::-1]

    positions = np.linspace(0, 255, len(bgr))
    x = np.arange(256)

    lut = np.zeros((256, 3), dtype=np.uint8)

    for c in range(3):
        lut[:, c] = np.interp(
            x,
            positions,
            bgr[:, c]
        ).astype(np.uint8)

    return lut


THERMAL_LUT = make_reference_lut()


def thermal_color(gray):
    """
    Correct replacement for cv2.LUT() when using a 256x3 LUT.
    Avoids the OpenCV channel assertion error.
    """
    gray = np.asarray(gray, dtype=np.uint8)
    return THERMAL_LUT[gray].copy()


# ============================================================
# FRAME DETAIL
# ============================================================

def prepare_gray(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # CLAHE keeps wall/floor/door structure visible while
    # removing the original RGB appearance.
    clahe = cv2.createCLAHE(
        clipLimit=1.8,
        tileGridSize=(8, 8)
    )

    gray = clahe.apply(gray)

    return gray


# ============================================================
# COOL THERMAL BACKGROUND
# ============================================================

def make_cool_background(frame):
    """
    Reference-style REALISTIC SYNTHETIC THERMAL BACKGROUND.

    ONLY the background rendering is changed here.

    Goal:
      - Preserve walls, doors, furniture and room structure.
      - Remove the normal RGB appearance.
      - Make the whole non-detected scene look like a real
        low-resolution thermal-camera image.
      - Keep the background predominantly dark/cool blue.
      - Allow natural blue -> cyan variations instead of making
        the entire background a flat dark blue.

    Hot PERSON/LAPTOP/MOBILE rendering is handled elsewhere and
    is intentionally unchanged.
    """
    h, w = frame.shape[:2]

    gray = prepare_gray(frame)

    # --------------------------------------------------------
    # Thermal cameras have much lower spatial detail than RGB.
    # Build a smooth low-frequency temperature field.
    # --------------------------------------------------------

    low_w = max(64, w // 12)
    low_h = max(36, h // 12)

    low = cv2.resize(
        gray,
        (low_w, low_h),
        interpolation=cv2.INTER_AREA
    )

    low = cv2.GaussianBlur(
        low,
        (0, 0),
        2.2
    )

    field = cv2.resize(
        low,
        (w, h),
        interpolation=cv2.INTER_CUBIC
    ).astype(np.float32)

    # --------------------------------------------------------
    # Local thermal texture.
    # This keeps walls/doors/furniture recognizable without
    # making them look like a normal RGB image.
    # --------------------------------------------------------

    smooth = cv2.GaussianBlur(
        gray.astype(np.float32),
        (0, 0),
        10
    )

    detail = gray.astype(np.float32) - smooth

    # Very subtle detail, similar to a real thermal sensor.
    field += detail * 0.10

    # --------------------------------------------------------
    # Robust normalization.
    # Avoid extreme bright areas turning the background hot.
    # --------------------------------------------------------

    lo, hi = np.percentile(
        field,
        (3, 97)
    )

    if hi - lo < 1.0:
        normalized = np.zeros_like(field)
    else:
        normalized = (
            field - lo
        ) / (
            hi - lo
        )

    normalized = np.clip(
        normalized,
        0.0,
        1.0
    )

    # --------------------------------------------------------
    # DARK NATURAL THERMAL BACKGROUND
    #
    # Most of the scene:
    #     deep blue -> blue
    #
    # Naturally warmer/correctly illuminated regions:
    #     blue -> cyan
    #
    # IMPORTANT:
    # Background does NOT reach green/yellow/red.
    # Those colors remain reserved for detected hot objects.
    # --------------------------------------------------------

    background_intensity = (
        4.0
        +
        normalized * 66.0
    )

    # Preserve subtle vertical/horizontal thermal drift so a wall
    # does not look like a completely flat digital fill.
    yy = np.linspace(
        -1.0,
        1.0,
        h,
        dtype=np.float32
    )[:, None]

    xx = np.linspace(
        -1.0,
        1.0,
        w,
        dtype=np.float32
    )[None, :]

    ambient_gradient = (
        2.2 * (1.0 - np.abs(xx))
        +
        1.4 * (1.0 - np.abs(yy))
    )

    background_intensity += ambient_gradient

    # Tiny sensor-like noise.
    rng = np.random.default_rng()
    noise = rng.normal(
        0.0,
        0.8,
        (h, w)
    ).astype(np.float32)

    background_intensity += noise

    # Strictly cool ceiling.
    background_intensity = np.clip(
        background_intensity,
        2,
        76
    ).astype(np.uint8)

    background = thermal_color(
        background_intensity
    )

    # --------------------------------------------------------
    # Thermal-camera softness.
    # --------------------------------------------------------

    background = cv2.GaussianBlur(
        background,
        (5, 5),
        0
    )

    # Very mild local contrast; keeps wall boundaries visible.
    lab = cv2.cvtColor(
        background,
        cv2.COLOR_BGR2LAB
    )

    l, a, b = cv2.split(lab)

    l = cv2.normalize(
        l,
        None,
        8,
        115,
        cv2.NORM_MINMAX
    ).astype(np.uint8)

    # Do not allow this operation to create hot background colors.
    l = np.clip(
        l,
        8,
        115
    ).astype(np.uint8)

    background = cv2.cvtColor(
        cv2.merge((l, a, b)),
        cv2.COLOR_LAB2BGR
    )

    return background, gray


# ============================================================
# MASK UTILITIES
# ============================================================

def resize_mask(mask):
    mask = cv2.resize(
        mask.astype(np.float32),
        (FRAME_WIDTH, FRAME_HEIGHT),
        interpolation=cv2.INTER_LINEAR
    )

    return np.clip(mask, 0.0, 1.0)


def mask_geometry(mask):
    binary = (
        mask > 0.45
    ).astype(np.uint8)

    ys, xs = np.where(binary > 0)

    if len(xs) == 0:
        return None

    x1 = int(xs.min())
    y1 = int(ys.min())
    x2 = int(xs.max())
    y2 = int(ys.max())

    width = max(
        1,
        x2 - x1 + 1
    )

    height = max(
        1,
        y2 - y1 + 1
    )

    yy, xx = np.mgrid[
        0:mask.shape[0],
        0:mask.shape[1]
    ]

    rx = (
        xx - x1
    ) / width

    ry = (
        yy - y1
    ) / height

    distance = cv2.distanceTransform(
        binary,
        cv2.DIST_L2,
        5
    )

    if distance.max() > 0:
        distance /= distance.max()

    return (
        binary,
        rx,
        ry,
        distance,
        x1,
        y1,
        x2,
        y2
    )


# ============================================================
# HUMAN THERMAL MODEL
# ============================================================

def human_heat(mask, temperature, gray):
    """
    High-detail synthetic human thermal rendering.

    The segmentation mask is respected, while the heat map uses
    body-relative hot zones so a person looks like a thermal body
    rather than a solid colored silhouette.
    """
    h, w = mask.shape
    geo = mask_geometry(mask)

    if geo is None:
        return np.zeros((h, w), dtype=np.uint8)

    (
        binary,
        rx,
        ry,
        distance,
        x1,
        y1,
        x2,
        y2
    ) = geo

    # Multiple overlapping Gaussian heat zones.
    head = np.exp(
        -(
            ((rx - 0.50) / 0.16) ** 2 +
            ((ry - 0.105) / 0.105) ** 2
        )
    )

    face = np.exp(
        -(
            ((rx - 0.50) / 0.105) ** 2 +
            ((ry - 0.175) / 0.080) ** 2
        )
    )

    neck = np.exp(
        -(
            ((rx - 0.50) / 0.105) ** 2 +
            ((ry - 0.275) / 0.100) ** 2
        )
    )

    chest = np.exp(
        -(
            ((rx - 0.50) / 0.255) ** 2 +
            ((ry - 0.43) / 0.245) ** 2
        )
    )

    abdomen = np.exp(
        -(
            ((rx - 0.50) / 0.235) ** 2 +
            ((ry - 0.62) / 0.260) ** 2
        )
    )

    left_hand = np.exp(
        -(
            ((rx - 0.23) / 0.095) ** 2 +
            ((ry - 0.48) / 0.125) ** 2
        )
    )

    right_hand = np.exp(
        -(
            ((rx - 0.77) / 0.095) ** 2 +
            ((ry - 0.48) / 0.125) ** 2
        )
    )

    left_arm = np.exp(
        -(
            ((rx - 0.31) / 0.12) ** 2 +
            ((ry - 0.48) / 0.31) ** 2
        )
    )

    right_arm = np.exp(
        -(
            ((rx - 0.69) / 0.12) ** 2 +
            ((ry - 0.48) / 0.31) ** 2
        )
    )

    # Central body is warmer than limbs.
    core = np.exp(
        -(
            ((rx - 0.50) / 0.31) ** 2 +
            ((ry - 0.52) / 0.44) ** 2
        )
    )

    # Extremities are naturally cooler.
    extremity_cool = (
        0.22 *
        (
            np.abs(rx - 0.50)
            +
            np.maximum(ry - 0.65, 0)
        )
    )

    heat = (
        0.20 * distance +
        0.42 * head +
        0.48 * face +
        0.22 * neck +
        0.30 * chest +
        0.18 * abdomen +
        0.16 * left_hand +
        0.16 * right_hand +
        0.08 * left_arm +
        0.08 * right_arm +
        0.12 * core -
        extremity_cool
    )

    # Fine thermal texture.
    local = cv2.GaussianBlur(
        gray.astype(np.float32),
        (0, 0),
        4
    )

    local = cv2.normalize(
        local,
        None,
        0,
        1,
        cv2.NORM_MINMAX
    )

    heat += local * 0.035

    low, high = TEMP_RANGES["PERSON"]

    temp_factor = (
        temperature - low
    ) / (
        high - low
    )

    temp_factor = np.clip(
        temp_factor,
        0,
        1
    )

    # Higher base intensity ensures the person is clearly hotter
    # than the dark-blue background.
    intensity = (
        112 +
        temp_factor * 78 +
        heat * 92
    )

    intensity = np.clip(
        intensity,
        0,
        255
    ).astype(np.uint8)

    # Thermal-camera blur, not cartoon-flat edges.
    intensity = cv2.GaussianBlur(
        intensity,
        (9, 9),
        0
    )

    # Make sure mask interior remains thermally visible.
    intensity = np.where(
        binary > 0,
        intensity,
        0
    ).astype(np.uint8)

    return intensity


# ============================================================
# ELECTRONIC THERMAL MODEL
# ============================================================

def electronic_heat(mask, temperature, device_type, gray):
    """
    More accurate thermal distribution for LAPTOP/MOBILE and
    thermal-only electronics.

    Laptop:
        CPU/GPU/VRM/vent zones are hottest.

    Mobile:
        SoC/processor zone and battery zone are hotter.

    Other electronics:
        receive a softer generalized heat map.
    """
    h, w = mask.shape
    geo = mask_geometry(mask)

    if geo is None:
        return np.zeros((h, w), dtype=np.uint8)

    (
        binary,
        rx,
        ry,
        distance,
        x1,
        y1,
        x2,
        y2
    ) = geo

    if device_type == "LAPTOP":

        # CPU / SoC region.
        cpu = np.exp(
            -(
                ((rx - 0.48) / 0.15) ** 2 +
                ((ry - 0.70) / 0.13) ** 2
            )
        )

        # GPU region.
        gpu = np.exp(
            -(
                ((rx - 0.69) / 0.14) ** 2 +
                ((ry - 0.70) / 0.13) ** 2
            )
        )

        # VRM/power area.
        vrm = np.exp(
            -(
                ((rx - 0.35) / 0.12) ** 2 +
                ((ry - 0.72) / 0.13) ** 2
            )
        )

        # Exhaust/vent heat.
        vent = np.exp(
            -(
                ((rx - 0.83) / 0.12) ** 2 +
                ((ry - 0.78) / 0.09) ** 2
            )
        )

        # Keyboard is warm but not as hot as internals.
        keyboard = np.exp(
            -(
                ((rx - 0.50) / 0.39) ** 2 +
                ((ry - 0.55) / 0.13) ** 2
            )
        )

        # Hinge area.
        hinge = np.exp(
            -(
                ((rx - 0.50) / 0.34) ** 2 +
                ((ry - 0.38) / 0.08) ** 2
            )
        )

        heat = (
            0.18 * distance +
            0.58 * cpu +
            0.43 * gpu +
            0.30 * vrm +
            0.32 * vent +
            0.10 * keyboard +
            0.10 * hinge
        )

    elif device_type == "MOBILE":

        # Processor / SoC.
        processor = np.exp(
            -(
                ((rx - 0.50) / 0.15) ** 2 +
                ((ry - 0.34) / 0.14) ** 2
            )
        )

        # Battery.
        battery = np.exp(
            -(
                ((rx - 0.50) / 0.25) ** 2 +
                ((ry - 0.67) / 0.21) ** 2
            )
        )

        # Camera/upper electronics area.
        camera = np.exp(
            -(
                ((rx - 0.78) / 0.12) ** 2 +
                ((ry - 0.12) / 0.10) ** 2
            )
        )

        # Charging/power area.
        power = np.exp(
            -(
                ((rx - 0.50) / 0.13) ** 2 +
                ((ry - 0.86) / 0.10) ** 2
            )
        )

        heat = (
            0.20 * distance +
            0.58 * processor +
            0.35 * battery +
            0.16 * camera +
            0.18 * power
        )

    elif device_type == "TV":

        screen = np.exp(
            -(
                ((rx - 0.50) / 0.39) ** 2 +
                ((ry - 0.48) / 0.30) ** 2
            )
        )

        electronics = np.exp(
            -(
                ((rx - 0.50) / 0.20) ** 2 +
                ((ry - 0.78) / 0.14) ** 2
            )
        )

        heat = (
            0.16 * distance +
            0.28 * screen +
            0.52 * electronics
        )

    else:

        center = np.exp(
            -(
                ((rx - 0.50) / 0.25) ** 2 +
                ((ry - 0.50) / 0.25) ** 2
            )
        )

        heat = (
            0.20 * distance +
            0.64 * center
        )

    # Slight natural surface variation.
    local = cv2.GaussianBlur(
        gray.astype(np.float32),
        (0, 0),
        3
    )

    local = cv2.normalize(
        local,
        None,
        0,
        1,
        cv2.NORM_MINMAX
    )

    heat += local * 0.03

    low, high = TEMP_RANGES.get(
        device_type,
        (28.0, 40.0)
    )

    temp_factor = (
        temperature - low
    ) / (
        high - low
    )

    temp_factor = np.clip(
        temp_factor,
        0,
        1
    )

    # Keep devices clearly hotter than the cool background.
    intensity = (
        98 +
        temp_factor * 95 +
        heat * 90
    )

    intensity = np.clip(
        intensity,
        0,
        255
    ).astype(np.uint8)

    intensity = cv2.GaussianBlur(
        intensity,
        (7, 7),
        0
    )

    intensity = np.where(
        binary > 0,
        intensity,
        0
    ).astype(np.uint8)

    return intensity


# ============================================================
# THERMAL COMPOSITING
# ============================================================

def composite_heat(
    base,
    heat_map,
    mask
):
    # Soft edge, but still tightly follows segmentation.
    soft = cv2.GaussianBlur(
        mask.astype(np.float32),
        (7, 7),
        0
    )

    soft = np.clip(
        soft,
        0,
        1
    )

    colored = thermal_color(
        heat_map
    )

    alpha = soft[:, :, None]

    result = (
        colored.astype(np.float32) * alpha
        +
        base.astype(np.float32) * (1.0 - alpha)
    )

    return np.clip(
        result,
        0,
        255
    ).astype(np.uint8)


# ============================================================
# REFERENCE-STYLE DASHED BOX
# ============================================================

def dashed_box(
    image,
    x1,
    y1,
    x2,
    y2
):
    white = (255, 255, 255)

    thickness = 2
    dash = 12
    gap = 8

    # Top.
    x = x1
    while x < x2:
        xe = min(
            x + dash,
            x2
        )
        cv2.line(
            image,
            (x, y1),
            (xe, y1),
            white,
            thickness,
            cv2.LINE_AA
        )
        x += dash + gap

    # Bottom.
    x = x1
    while x < x2:
        xe = min(
            x + dash,
            x2
        )
        cv2.line(
            image,
            (x, y2),
            (xe, y2),
            white,
            thickness,
            cv2.LINE_AA
        )
        x += dash + gap

    # Left.
    y = y1
    while y < y2:
        ye = min(
            y + dash,
            y2
        )
        cv2.line(
            image,
            (x1, y),
            (x1, ye),
            white,
            thickness,
            cv2.LINE_AA
        )
        y += dash + gap

    # Right.
    y = y1
    while y < y2:
        ye = min(
            y + dash,
            y2
        )
        cv2.line(
            image,
            (x2, y),
            (x2, ye),
            white,
            thickness,
            cv2.LINE_AA
        )
        y += dash + gap


# ============================================================
# LABEL
# ============================================================

def draw_detection_label(
    image,
    name,
    number,
    percentage,
    temperature,
    x,
    y
):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.52
    thickness = 1

    part1 = f"{name} {number}"
    part2 = f"{percentage}%"
    part3 = f"{temperature:.1f}°C"

    s1 = cv2.getTextSize(
        part1,
        font,
        scale,
        thickness
    )[0]

    s2 = cv2.getTextSize(
        part2,
        font,
        scale,
        thickness
    )[0]

    s3 = cv2.getTextSize(
        part3,
        font,
        scale,
        thickness
    )[0]

    gap = 8
    pad_x = 7
    pad_y = 6

    total_width = (
        s1[0]
        +
        s2[0]
        +
        s3[0]
        +
        gap * 2
        +
        pad_x * 2
    )

    total_height = (
        max(
            s1[1],
            s2[1],
            s3[1]
        )
        +
        pad_y * 2
    )

    bx1 = int(x)
    by2 = int(y)
    by1 = by2 - total_height
    bx2 = bx1 + total_width

    # Keep label inside frame.
    if bx2 > FRAME_WIDTH - 5:
        bx2 = FRAME_WIDTH - 5
        bx1 = max(
            4,
            bx2 - total_width
        )

    if by1 < 5:
        by1 = 5
        by2 = by1 + total_height
        text_y = by1 + total_height - pad_y - 1
    else:
        text_y = by2 - pad_y - 1

    # Dark navy panel.
    overlay = image.copy()

    cv2.rectangle(
        overlay,
        (bx1, by1),
        (bx2, by2),
        (1, 14, 42),
        -1
    )

    cv2.addWeighted(
        overlay,
        0.90,
        image,
        0.10,
        0,
        image
    )

    # Thin cyan border.
    cv2.rectangle(
        image,
        (bx1, by1),
        (bx2, by2),
        (70, 235, 255),
        1,
        cv2.LINE_AA
    )

    tx = bx1 + pad_x

    cv2.putText(
        image,
        part1,
        (tx, text_y),
        font,
        scale,
        (235, 255, 255),
        thickness,
        cv2.LINE_AA
    )

    tx += s1[0] + gap

    cv2.putText(
        image,
        part2,
        (tx, text_y),
        font,
        scale,
        (0, 150, 255),
        thickness,
        cv2.LINE_AA
    )

    tx += s2[0] + gap

    cv2.putText(
        image,
        part3,
        (tx, text_y),
        font,
        scale,
        (235, 255, 255),
        thickness,
        cv2.LINE_AA
    )


# ============================================================
# TEMPERATURE SCALE
# ============================================================

def draw_temperature_scale(image):

    # Reference-style right-side scale.
    scale_w = max(
        18,
        int(FRAME_WIDTH * 0.014)
    )

    scale_h = int(
        FRAME_HEIGHT * 0.55
    )

    x = FRAME_WIDTH - int(
        FRAME_WIDTH * 0.075
    )

    y = int(
        FRAME_HEIGHT * 0.145
    )

    # Gradient from hot at top to cold at bottom.
    values = np.linspace(
        255,
        0,
        scale_h
    ).astype(np.uint8)

    values = np.repeat(
        values[:, None],
        scale_w,
        axis=1
    )

    gradient = thermal_color(
        values
    )

    # Outer cyan panel.
    cv2.rectangle(
        image,
        (x - 7, y - 7),
        (
            x + scale_w + 7,
            y + scale_h + 7
        ),
        (70, 235, 255),
        1,
        cv2.LINE_AA
    )

    image[
        y:y + scale_h,
        x:x + scale_w
    ] = gradient

    font = cv2.FONT_HERSHEY_SIMPLEX

    cv2.putText(
        image,
        "55°C",
        (
            x - 5,
            y - 14
        ),
        font,
        0.50,
        (235, 255, 255),
        1,
        cv2.LINE_AA
    )

    cv2.putText(
        image,
        "20°C",
        (
            x - 5,
            y + scale_h + 25
        ),
        font,
        0.50,
        (235, 255, 255),
        1,
        cv2.LINE_AA
    )


# ============================================================
# BOTTOM UI
# ============================================================

def draw_bottom_ui(image):

    # THERMAL VISION
    left = 30
    bottom = FRAME_HEIGHT - 18

    box_w = 245
    box_h = 38

    top = bottom - box_h

    cv2.rectangle(
        image,
        (left, top),
        (left + box_w, bottom),
        (1, 14, 42),
        -1
    )

    cv2.rectangle(
        image,
        (left, top),
        (left + box_w, bottom),
        (60, 230, 255),
        1,
        cv2.LINE_AA
    )

    cv2.putText(
        image,
        "THERMAL VISION",
        (left + 13, top + 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.57,
        (175, 255, 255),
        1,
        cv2.LINE_AA
    )

    # LIVE
    live_w = 130
    live_h = 38

    live_left = FRAME_WIDTH - 30 - live_w
    live_top = FRAME_HEIGHT - 18 - live_h

    cv2.rectangle(
        image,
        (live_left, live_top),
        (
            live_left + live_w,
            live_top + live_h
        ),
        (1, 14, 42),
        -1
    )

    cv2.rectangle(
        image,
        (live_left, live_top),
        (
            live_left + live_w,
            live_top + live_h
        ),
        (60, 230, 255),
        1,
        cv2.LINE_AA
    )

    cv2.circle(
        image,
        (
            live_left + 18,
            live_top + 19
        ),
        7,
        (0, 0, 255),
        -1,
        cv2.LINE_AA
    )

    cv2.putText(
        image,
        "LIVE",
        (
            live_left + 33,
            live_top + 26
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (190, 255, 255),
        1,
        cv2.LINE_AA
    )


# ============================================================
# STABLE SYNTHETIC TEMPERATURE
# ============================================================

temperature_cache = {}


def get_temperature(
    object_name,
    track_id,
    x1,
    y1
):
    key = (
        object_name,
        track_id if track_id is not None else (
            x1 // 12,
            y1 // 12
        )
    )

    if key in temperature_cache:
        return temperature_cache[key]

    low, high = TEMP_RANGES[
        object_name
    ]

    # Deterministic value: stable instead of jumping randomly.
    seed = (
        abs(hash(key)) % 10000
    )

    fraction = (
        seed % 1000
    ) / 1000.0

    # Keep people in realistic human range.
    temperature = (
        low
        +
        fraction * (high - low)
    )

    temperature_cache[key] = float(
        temperature
    )

    # Prevent unlimited cache growth.
    if len(temperature_cache) > 1000:
        temperature_cache.clear()

    return float(
        temperature
    )


def get_percentage(
    object_name,
    temperature
):
    low, high = TEMP_RANGES[
        object_name
    ]

    raw = (
        temperature - low
    ) / (
        high - low
    )

    raw = np.clip(
        raw,
        0,
        1
    )

    # Reference-like display ranges.
    if object_name == "PERSON":
        value = 72 + raw * 18

    elif object_name == "LAPTOP":
        value = 82 + raw * 14

    elif object_name == "MOBILE":
        value = 78 + raw * 14

    else:
        value = 55 + raw * 25

    return int(
        np.clip(
            value,
            1,
            99
        )
    )



import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from ultralytics import YOLO

# ============================================================
# LIVE SYNTHETIC THERMAL CAMERA
# ============================================================
# Visual target: the supplied thermal-camera reference image.
#
# IMPORTANT:
# - The reference image is NOT used at runtime.
# - Your LIVE RGB webcam is transformed frame-by-frame.
# - Background stays cool blue/cyan.
# - PERSON / LAPTOP / MOBILE become hot thermal objects.
# - Other supported electronic objects receive heat only.
# - Only PERSON/LAPTOP/MOBILE receive labels + dashed boxes.
#
# This is SYNTHETIC thermal visualization. A normal RGB webcam
# cannot physically measure °C.
# ============================================================

MODEL_NAME = "yolo11n-seg.pt"
CAMERA_INDEX = 0

CONFIDENCE = 0.30
IOU = 0.50

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

WINDOW_NAME = "THERMAL VISION"

# Use CUDA automatically when available.
DEVICE = 0 if torch.cuda.is_available() else "cpu"

# ============================================================
# TEMPERATURE RANGES
# ============================================================

TEMP_RANGES = {
    "PERSON": (35.8, 38.2),
    "LAPTOP": (40.0, 50.0),
    "MOBILE": (34.0, 44.0),

    # Thermal-only electronics
    "TV": (30.0, 40.0),
    "MOUSE": (28.0, 35.0),
    "REMOTE": (27.0, 34.0),
    "KEYBOARD": (29.0, 38.0),
}

BACKGROUND_TEMP = (20.0, 29.0)

# ============================================================
# COCO CLASS IDs
# ============================================================

PERSON_ID = 0
TV_ID = 62
LAPTOP_ID = 63
MOUSE_ID = 64
REMOTE_ID = 65
KEYBOARD_ID = 66
MOBILE_ID = 67

LABELED = {
    PERSON_ID: "PERSON",
    LAPTOP_ID: "LAPTOP",
    MOBILE_ID: "MOBILE",
}

THERMAL_ONLY = {
    TV_ID: "TV",
    MOUSE_ID: "MOUSE",
    REMOTE_ID: "REMOTE",
    KEYBOARD_ID: "KEYBOARD",
}


# ============================================================
# REFERENCE-STYLE THERMAL LUT
# Colors sampled/approximated from the supplied reference image.
# OpenCV BGR order.
# ============================================================

def make_reference_lut():
    # Deep blue -> electric blue -> cyan -> green -> yellow
    # -> orange -> red -> white.
    rgb = np.array([
        [0,   18,  72],
        [0,   25, 176],
        [0,   44, 216],
        [0,   81, 248],
        [2,  150, 249],
        [8,  220, 191],
        [18, 245,  67],
        [130,249,  6],
        [237,236,  4],
        [252,127,  7],
        [250, 26,  1],
        [210,  0,  0],
        [255, 70, 70],
        [255,255,255],
    ], dtype=np.uint8)

    # Convert RGB palette to BGR for OpenCV.
    bgr = rgb[:, ::-1]

    positions = np.linspace(0, 255, len(bgr))
    x = np.arange(256)

    lut = np.zeros((256, 3), dtype=np.uint8)

    for c in range(3):
        lut[:, c] = np.interp(
            x,
            positions,
            bgr[:, c]
        ).astype(np.uint8)

    return lut


THERMAL_LUT = make_reference_lut()


def thermal_color(gray):
    """
    Correct replacement for cv2.LUT() when using a 256x3 LUT.
    Avoids the OpenCV channel assertion error.
    """
    gray = np.asarray(gray, dtype=np.uint8)
    return THERMAL_LUT[gray].copy()


# ============================================================
# FRAME DETAIL
# ============================================================

def prepare_gray(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # CLAHE keeps wall/floor/door structure visible while
    # removing the original RGB appearance.
    clahe = cv2.createCLAHE(
        clipLimit=1.8,
        tileGridSize=(8, 8)
    )

    gray = clahe.apply(gray)

    return gray


# ============================================================
# COOL THERMAL BACKGROUND
# ============================================================

def make_cool_background(frame):
    """
    Reference-style REALISTIC SYNTHETIC THERMAL BACKGROUND.

    ONLY the background rendering is changed here.

    Goal:
      - Preserve walls, doors, furniture and room structure.
      - Remove the normal RGB appearance.
      - Make the whole non-detected scene look like a real
        low-resolution thermal-camera image.
      - Keep the background predominantly dark/cool blue.
      - Allow natural blue -> cyan variations instead of making
        the entire background a flat dark blue.

    Hot PERSON/LAPTOP/MOBILE rendering is handled elsewhere and
    is intentionally unchanged.
    """
    h, w = frame.shape[:2]

    gray = prepare_gray(frame)

    # --------------------------------------------------------
    # Thermal cameras have much lower spatial detail than RGB.
    # Build a smooth low-frequency temperature field.
    # --------------------------------------------------------

    low_w = max(64, w // 12)
    low_h = max(36, h // 12)

    low = cv2.resize(
        gray,
        (low_w, low_h),
        interpolation=cv2.INTER_AREA
    )

    low = cv2.GaussianBlur(
        low,
        (0, 0),
        2.2
    )

    field = cv2.resize(
        low,
        (w, h),
        interpolation=cv2.INTER_CUBIC
    ).astype(np.float32)

    # --------------------------------------------------------
    # Local thermal texture.
    # This keeps walls/doors/furniture recognizable without
    # making them look like a normal RGB image.
    # --------------------------------------------------------

    smooth = cv2.GaussianBlur(
        gray.astype(np.float32),
        (0, 0),
        10
    )

    detail = gray.astype(np.float32) - smooth

    # Very subtle detail, similar to a real thermal sensor.
    field += detail * 0.10

    # --------------------------------------------------------
    # Robust normalization.
    # Avoid extreme bright areas turning the background hot.
    # --------------------------------------------------------

    lo, hi = np.percentile(
        field,
        (3, 97)
    )

    if hi - lo < 1.0:
        normalized = np.zeros_like(field)
    else:
        normalized = (
            field - lo
        ) / (
            hi - lo
        )

    normalized = np.clip(
        normalized,
        0.0,
        1.0
    )

    # --------------------------------------------------------
    # DARK NATURAL THERMAL BACKGROUND
    #
    # Most of the scene:
    #     deep blue -> blue
    #
    # Naturally warmer/correctly illuminated regions:
    #     blue -> cyan
    #
    # IMPORTANT:
    # Background does NOT reach green/yellow/red.
    # Those colors remain reserved for detected hot objects.
    # --------------------------------------------------------

    background_intensity = (
        4.0
        +
        normalized * 66.0
    )

    # Preserve subtle vertical/horizontal thermal drift so a wall
    # does not look like a completely flat digital fill.
    yy = np.linspace(
        -1.0,
        1.0,
        h,
        dtype=np.float32
    )[:, None]

    xx = np.linspace(
        -1.0,
        1.0,
        w,
        dtype=np.float32
    )[None, :]

    ambient_gradient = (
        2.2 * (1.0 - np.abs(xx))
        +
        1.4 * (1.0 - np.abs(yy))
    )

    background_intensity += ambient_gradient

    # Tiny sensor-like noise.
    rng = np.random.default_rng()
    noise = rng.normal(
        0.0,
        0.8,
        (h, w)
    ).astype(np.float32)

    background_intensity += noise

    # Strictly cool ceiling.
    background_intensity = np.clip(
        background_intensity,
        2,
        76
    ).astype(np.uint8)

    background = thermal_color(
        background_intensity
    )

    # --------------------------------------------------------
    # Thermal-camera softness.
    # --------------------------------------------------------

    background = cv2.GaussianBlur(
        background,
        (5, 5),
        0
    )

    # Very mild local contrast; keeps wall boundaries visible.
    lab = cv2.cvtColor(
        background,
        cv2.COLOR_BGR2LAB
    )

    l, a, b = cv2.split(lab)

    l = cv2.normalize(
        l,
        None,
        8,
        115,
        cv2.NORM_MINMAX
    ).astype(np.uint8)

    # Do not allow this operation to create hot background colors.
    l = np.clip(
        l,
        8,
        115
    ).astype(np.uint8)

    background = cv2.cvtColor(
        cv2.merge((l, a, b)),
        cv2.COLOR_LAB2BGR
    )

    return background, gray


# ============================================================
# MASK UTILITIES
# ============================================================

def resize_mask(mask):
    mask = cv2.resize(
        mask.astype(np.float32),
        (FRAME_WIDTH, FRAME_HEIGHT),
        interpolation=cv2.INTER_LINEAR
    )

    return np.clip(mask, 0.0, 1.0)


def mask_geometry(mask):
    binary = (
        mask > 0.45
    ).astype(np.uint8)

    ys, xs = np.where(binary > 0)

    if len(xs) == 0:
        return None

    x1 = int(xs.min())
    y1 = int(ys.min())
    x2 = int(xs.max())
    y2 = int(ys.max())

    width = max(
        1,
        x2 - x1 + 1
    )

    height = max(
        1,
        y2 - y1 + 1
    )

    yy, xx = np.mgrid[
        0:mask.shape[0],
        0:mask.shape[1]
    ]

    rx = (
        xx - x1
    ) / width

    ry = (
        yy - y1
    ) / height

    distance = cv2.distanceTransform(
        binary,
        cv2.DIST_L2,
        5
    )

    if distance.max() > 0:
        distance /= distance.max()

    return (
        binary,
        rx,
        ry,
        distance,
        x1,
        y1,
        x2,
        y2
    )


# ============================================================
# HUMAN THERMAL MODEL
# ============================================================

def human_heat(mask, temperature, gray):
    """
    High-detail synthetic human thermal rendering.

    The segmentation mask is respected, while the heat map uses
    body-relative hot zones so a person looks like a thermal body
    rather than a solid colored silhouette.
    """
    h, w = mask.shape
    geo = mask_geometry(mask)

    if geo is None:
        return np.zeros((h, w), dtype=np.uint8)

    (
        binary,
        rx,
        ry,
        distance,
        x1,
        y1,
        x2,
        y2
    ) = geo

    # Multiple overlapping Gaussian heat zones.
    head = np.exp(
        -(
            ((rx - 0.50) / 0.16) ** 2 +
            ((ry - 0.105) / 0.105) ** 2
        )
    )

    face = np.exp(
        -(
            ((rx - 0.50) / 0.105) ** 2 +
            ((ry - 0.175) / 0.080) ** 2
        )
    )

    neck = np.exp(
        -(
            ((rx - 0.50) / 0.105) ** 2 +
            ((ry - 0.275) / 0.100) ** 2
        )
    )

    chest = np.exp(
        -(
            ((rx - 0.50) / 0.255) ** 2 +
            ((ry - 0.43) / 0.245) ** 2
        )
    )

    abdomen = np.exp(
        -(
            ((rx - 0.50) / 0.235) ** 2 +
            ((ry - 0.62) / 0.260) ** 2
        )
    )

    left_hand = np.exp(
        -(
            ((rx - 0.23) / 0.095) ** 2 +
            ((ry - 0.48) / 0.125) ** 2
        )
    )

    right_hand = np.exp(
        -(
            ((rx - 0.77) / 0.095) ** 2 +
            ((ry - 0.48) / 0.125) ** 2
        )
    )

    left_arm = np.exp(
        -(
            ((rx - 0.31) / 0.12) ** 2 +
            ((ry - 0.48) / 0.31) ** 2
        )
    )

    right_arm = np.exp(
        -(
            ((rx - 0.69) / 0.12) ** 2 +
            ((ry - 0.48) / 0.31) ** 2
        )
    )

    # Central body is warmer than limbs.
    core = np.exp(
        -(
            ((rx - 0.50) / 0.31) ** 2 +
            ((ry - 0.52) / 0.44) ** 2
        )
    )

    # Extremities are naturally cooler.
    extremity_cool = (
        0.22 *
        (
            np.abs(rx - 0.50)
            +
            np.maximum(ry - 0.65, 0)
        )
    )

    heat = (
        0.20 * distance +
        0.42 * head +
        0.48 * face +
        0.22 * neck +
        0.30 * chest +
        0.18 * abdomen +
        0.16 * left_hand +
        0.16 * right_hand +
        0.08 * left_arm +
        0.08 * right_arm +
        0.12 * core -
        extremity_cool
    )

    # Fine thermal texture.
    local = cv2.GaussianBlur(
        gray.astype(np.float32),
        (0, 0),
        4
    )

    local = cv2.normalize(
        local,
        None,
        0,
        1,
        cv2.NORM_MINMAX
    )

    heat += local * 0.035

    low, high = TEMP_RANGES["PERSON"]

    temp_factor = (
        temperature - low
    ) / (
        high - low
    )

    temp_factor = np.clip(
        temp_factor,
        0,
        1
    )

    # Higher base intensity ensures the person is clearly hotter
    # than the dark-blue background.
    intensity = (
        112 +
        temp_factor * 78 +
        heat * 92
    )

    intensity = np.clip(
        intensity,
        0,
        255
    ).astype(np.uint8)

    # Thermal-camera blur, not cartoon-flat edges.
    intensity = cv2.GaussianBlur(
        intensity,
        (9, 9),
        0
    )

    # Make sure mask interior remains thermally visible.
    intensity = np.where(
        binary > 0,
        intensity,
        0
    ).astype(np.uint8)

    return intensity


# ============================================================
# ELECTRONIC THERMAL MODEL
# ============================================================

def electronic_heat(mask, temperature, device_type, gray):
    """
    More accurate thermal distribution for LAPTOP/MOBILE and
    thermal-only electronics.

    Laptop:
        CPU/GPU/VRM/vent zones are hottest.

    Mobile:
        SoC/processor zone and battery zone are hotter.

    Other electronics:
        receive a softer generalized heat map.
    """
    h, w = mask.shape
    geo = mask_geometry(mask)

    if geo is None:
        return np.zeros((h, w), dtype=np.uint8)

    (
        binary,
        rx,
        ry,
        distance,
        x1,
        y1,
        x2,
        y2
    ) = geo

    if device_type == "LAPTOP":

        # CPU / SoC region.
        cpu = np.exp(
            -(
                ((rx - 0.48) / 0.15) ** 2 +
                ((ry - 0.70) / 0.13) ** 2
            )
        )

        # GPU region.
        gpu = np.exp(
            -(
                ((rx - 0.69) / 0.14) ** 2 +
                ((ry - 0.70) / 0.13) ** 2
            )
        )

        # VRM/power area.
        vrm = np.exp(
            -(
                ((rx - 0.35) / 0.12) ** 2 +
                ((ry - 0.72) / 0.13) ** 2
            )
        )

        # Exhaust/vent heat.
        vent = np.exp(
            -(
                ((rx - 0.83) / 0.12) ** 2 +
                ((ry - 0.78) / 0.09) ** 2
            )
        )

        # Keyboard is warm but not as hot as internals.
        keyboard = np.exp(
            -(
                ((rx - 0.50) / 0.39) ** 2 +
                ((ry - 0.55) / 0.13) ** 2
            )
        )

        # Hinge area.
        hinge = np.exp(
            -(
                ((rx - 0.50) / 0.34) ** 2 +
                ((ry - 0.38) / 0.08) ** 2
            )
        )

        heat = (
            0.18 * distance +
            0.58 * cpu +
            0.43 * gpu +
            0.30 * vrm +
            0.32 * vent +
            0.10 * keyboard +
            0.10 * hinge
        )

    elif device_type == "MOBILE":

        # Processor / SoC.
        processor = np.exp(
            -(
                ((rx - 0.50) / 0.15) ** 2 +
                ((ry - 0.34) / 0.14) ** 2
            )
        )

        # Battery.
        battery = np.exp(
            -(
                ((rx - 0.50) / 0.25) ** 2 +
                ((ry - 0.67) / 0.21) ** 2
            )
        )

        # Camera/upper electronics area.
        camera = np.exp(
            -(
                ((rx - 0.78) / 0.12) ** 2 +
                ((ry - 0.12) / 0.10) ** 2
            )
        )

        # Charging/power area.
        power = np.exp(
            -(
                ((rx - 0.50) / 0.13) ** 2 +
                ((ry - 0.86) / 0.10) ** 2
            )
        )

        heat = (
            0.20 * distance +
            0.58 * processor +
            0.35 * battery +
            0.16 * camera +
            0.18 * power
        )

    elif device_type == "TV":

        screen = np.exp(
            -(
                ((rx - 0.50) / 0.39) ** 2 +
                ((ry - 0.48) / 0.30) ** 2
            )
        )

        electronics = np.exp(
            -(
                ((rx - 0.50) / 0.20) ** 2 +
                ((ry - 0.78) / 0.14) ** 2
            )
        )

        heat = (
            0.16 * distance +
            0.28 * screen +
            0.52 * electronics
        )

    else:

        center = np.exp(
            -(
                ((rx - 0.50) / 0.25) ** 2 +
                ((ry - 0.50) / 0.25) ** 2
            )
        )

        heat = (
            0.20 * distance +
            0.64 * center
        )

    # Slight natural surface variation.
    local = cv2.GaussianBlur(
        gray.astype(np.float32),
        (0, 0),
        3
    )

    local = cv2.normalize(
        local,
        None,
        0,
        1,
        cv2.NORM_MINMAX
    )

    heat += local * 0.03

    low, high = TEMP_RANGES.get(
        device_type,
        (28.0, 40.0)
    )

    temp_factor = (
        temperature - low
    ) / (
        high - low
    )

    temp_factor = np.clip(
        temp_factor,
        0,
        1
    )

    # Keep devices clearly hotter than the cool background.
    intensity = (
        98 +
        temp_factor * 95 +
        heat * 90
    )

    intensity = np.clip(
        intensity,
        0,
        255
    ).astype(np.uint8)

    intensity = cv2.GaussianBlur(
        intensity,
        (7, 7),
        0
    )

    intensity = np.where(
        binary > 0,
        intensity,
        0
    ).astype(np.uint8)

    return intensity


# ============================================================
# THERMAL COMPOSITING
# ============================================================

def composite_heat(
    base,
    heat_map,
    mask
):
    # Soft edge, but still tightly follows segmentation.
    soft = cv2.GaussianBlur(
        mask.astype(np.float32),
        (7, 7),
        0
    )

    soft = np.clip(
        soft,
        0,
        1
    )

    colored = thermal_color(
        heat_map
    )

    alpha = soft[:, :, None]

    result = (
        colored.astype(np.float32) * alpha
        +
        base.astype(np.float32) * (1.0 - alpha)
    )

    return np.clip(
        result,
        0,
        255
    ).astype(np.uint8)


# ============================================================
# REFERENCE-STYLE DASHED BOX
# ============================================================

def dashed_box(
    image,
    x1,
    y1,
    x2,
    y2
):
    white = (255, 255, 255)

    thickness = 2
    dash = 12
    gap = 8

    # Top.
    x = x1
    while x < x2:
        xe = min(
            x + dash,
            x2
        )
        cv2.line(
            image,
            (x, y1),
            (xe, y1),
            white,
            thickness,
            cv2.LINE_AA
        )
        x += dash + gap

    # Bottom.
    x = x1
    while x < x2:
        xe = min(
            x + dash,
            x2
        )
        cv2.line(
            image,
            (x, y2),
            (xe, y2),
            white,
            thickness,
            cv2.LINE_AA
        )
        x += dash + gap

    # Left.
    y = y1
    while y < y2:
        ye = min(
            y + dash,
            y2
        )
        cv2.line(
            image,
            (x1, y),
            (x1, ye),
            white,
            thickness,
            cv2.LINE_AA
        )
        y += dash + gap

    # Right.
    y = y1
    while y < y2:
        ye = min(
            y + dash,
            y2
        )
        cv2.line(
            image,
            (x2, y),
            (x2, ye),
            white,
            thickness,
            cv2.LINE_AA
        )
        y += dash + gap


# ============================================================
# LABEL
# ============================================================

def draw_detection_label(
    image,
    name,
    number,
    percentage,
    temperature,
    x,
    y
):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.52
    thickness = 1

    part1 = f"{name} {number}"
    part2 = f"{percentage}%"
    part3 = f"{temperature:.1f}°C"

    s1 = cv2.getTextSize(
        part1,
        font,
        scale,
        thickness
    )[0]

    s2 = cv2.getTextSize(
        part2,
        font,
        scale,
        thickness
    )[0]

    s3 = cv2.getTextSize(
        part3,
        font,
        scale,
        thickness
    )[0]

    gap = 8
    pad_x = 7
    pad_y = 6

    total_width = (
        s1[0]
        +
        s2[0]
        +
        s3[0]
        +
        gap * 2
        +
        pad_x * 2
    )

    total_height = (
        max(
            s1[1],
            s2[1],
            s3[1]
        )
        +
        pad_y * 2
    )

    bx1 = int(x)
    by2 = int(y)
    by1 = by2 - total_height
    bx2 = bx1 + total_width

    # Keep label inside frame.
    if bx2 > FRAME_WIDTH - 5:
        bx2 = FRAME_WIDTH - 5
        bx1 = max(
            4,
            bx2 - total_width
        )

    if by1 < 5:
        by1 = 5
        by2 = by1 + total_height
        text_y = by1 + total_height - pad_y - 1
    else:
        text_y = by2 - pad_y - 1

    # Dark navy panel.
    overlay = image.copy()

    cv2.rectangle(
        overlay,
        (bx1, by1),
        (bx2, by2),
        (1, 14, 42),
        -1
    )

    cv2.addWeighted(
        overlay,
        0.90,
        image,
        0.10,
        0,
        image
    )

    # Thin cyan border.
    cv2.rectangle(
        image,
        (bx1, by1),
        (bx2, by2),
        (70, 235, 255),
        1,
        cv2.LINE_AA
    )

    tx = bx1 + pad_x

    cv2.putText(
        image,
        part1,
        (tx, text_y),
        font,
        scale,
        (235, 255, 255),
        thickness,
        cv2.LINE_AA
    )

    tx += s1[0] + gap

    cv2.putText(
        image,
        part2,
        (tx, text_y),
        font,
        scale,
        (0, 150, 255),
        thickness,
        cv2.LINE_AA
    )

    tx += s2[0] + gap

    cv2.putText(
        image,
        part3,
        (tx, text_y),
        font,
        scale,
        (235, 255, 255),
        thickness,
        cv2.LINE_AA
    )


# ============================================================
# TEMPERATURE SCALE
# ============================================================

def draw_temperature_scale(image):

    # Reference-style right-side scale.
    scale_w = max(
        18,
        int(FRAME_WIDTH * 0.014)
    )

    scale_h = int(
        FRAME_HEIGHT * 0.55
    )

    x = FRAME_WIDTH - int(
        FRAME_WIDTH * 0.075
    )

    y = int(
        FRAME_HEIGHT * 0.145
    )

    # Gradient from hot at top to cold at bottom.
    values = np.linspace(
        255,
        0,
        scale_h
    ).astype(np.uint8)

    values = np.repeat(
        values[:, None],
        scale_w,
        axis=1
    )

    gradient = thermal_color(
        values
    )

    # Outer cyan panel.
    cv2.rectangle(
        image,
        (x - 7, y - 7),
        (
            x + scale_w + 7,
            y + scale_h + 7
        ),
        (70, 235, 255),
        1,
        cv2.LINE_AA
    )

    image[
        y:y + scale_h,
        x:x + scale_w
    ] = gradient

    font = cv2.FONT_HERSHEY_SIMPLEX

    cv2.putText(
        image,
        "55°C",
        (
            x - 5,
            y - 14
        ),
        font,
        0.50,
        (235, 255, 255),
        1,
        cv2.LINE_AA
    )

    cv2.putText(
        image,
        "20°C",
        (
            x - 5,
            y + scale_h + 25
        ),
        font,
        0.50,
        (235, 255, 255),
        1,
        cv2.LINE_AA
    )


# ============================================================
# BOTTOM UI
# ============================================================

def draw_bottom_ui(image):

    # THERMAL VISION
    left = 30
    bottom = FRAME_HEIGHT - 18

    box_w = 245
    box_h = 38

    top = bottom - box_h

    cv2.rectangle(
        image,
        (left, top),
        (left + box_w, bottom),
        (1, 14, 42),
        -1
    )

    cv2.rectangle(
        image,
        (left, top),
        (left + box_w, bottom),
        (60, 230, 255),
        1,
        cv2.LINE_AA
    )

    cv2.putText(
        image,
        "THERMAL VISION",
        (left + 13, top + 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.57,
        (175, 255, 255),
        1,
        cv2.LINE_AA
    )

    # LIVE
    live_w = 130
    live_h = 38

    live_left = FRAME_WIDTH - 30 - live_w
    live_top = FRAME_HEIGHT - 18 - live_h

    cv2.rectangle(
        image,
        (live_left, live_top),
        (
            live_left + live_w,
            live_top + live_h
        ),
        (1, 14, 42),
        -1
    )

    cv2.rectangle(
        image,
        (live_left, live_top),
        (
            live_left + live_w,
            live_top + live_h
        ),
        (60, 230, 255),
        1,
        cv2.LINE_AA
    )

    cv2.circle(
        image,
        (
            live_left + 18,
            live_top + 19
        ),
        7,
        (0, 0, 255),
        -1,
        cv2.LINE_AA
    )

    cv2.putText(
        image,
        "LIVE",
        (
            live_left + 33,
            live_top + 26
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (190, 255, 255),
        1,
        cv2.LINE_AA
    )


# ============================================================
# STABLE SYNTHETIC TEMPERATURE
# ============================================================

temperature_cache = {}


def get_temperature(
    object_name,
    track_id,
    x1,
    y1
):
    key = (
        object_name,
        track_id if track_id is not None else (
            x1 // 12,
            y1 // 12
        )
    )

    if key in temperature_cache:
        return temperature_cache[key]

    low, high = TEMP_RANGES[
        object_name
    ]

    # Deterministic value: stable instead of jumping randomly.
    seed = (
        abs(hash(key)) % 10000
    )

    fraction = (
        seed % 1000
    ) / 1000.0

    # Keep people in realistic human range.
    temperature = (
        low
        +
        fraction * (high - low)
    )

    temperature_cache[key] = float(
        temperature
    )

    # Prevent unlimited cache growth.
    if len(temperature_cache) > 1000:
        temperature_cache.clear()

    return float(
        temperature
    )


def get_percentage(
    object_name,
    temperature
):
    low, high = TEMP_RANGES[
        object_name
    ]

    raw = (
        temperature - low
    ) / (
        high - low
    )

    raw = np.clip(
        raw,
        0,
        1
    )

    # Reference-like display ranges.
    if object_name == "PERSON":
        value = 72 + raw * 18

    elif object_name == "LAPTOP":
        value = 82 + raw * 14

    elif object_name == "MOBILE":
        value = 78 + raw * 14

    else:
        value = 55 + raw * 25

    return int(
        np.clip(
            value,
            1,
            99
        )
    )


class ThermalNode(Node):
    """Subscribe, process, and publish all derived frames from one input."""

    def __init__(self):
        super().__init__("thermal_node")
        self.declare_parameter("input_topic", "/live_feed")
        self.declare_parameter("thermal_topic", "/thermal_feed")
        self.declare_parameter("marked_topic", "/live_marked_feed")
        self.declare_parameter("detections_topic", "/thermal/detections")
        self.declare_parameter("confidence", CONFIDENCE)
        self.declare_parameter("iou", IOU)
        self.declare_parameter("inference_size", 640)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("show_window", False)

        self.bridge = CvBridge()
        self.input_lock = threading.Lock()
        self.latest_frame = None
        self.latest_header = None
        self.show_window = bool(self.get_parameter("show_window").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.confidence = float(self.get_parameter("confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.inference_size = int(self.get_parameter("inference_size").value)
        self.device = DEVICE
        self.use_half = self.device != "cpu" and torch.cuda.is_available()

        self.thermal_pub = self.create_publisher(
            CompressedImage,
            self.get_parameter("thermal_topic").value,
            qos_profile_sensor_data,
        )
        self.marked_pub = self.create_publisher(
            CompressedImage,
            self.get_parameter("marked_topic").value,
            qos_profile_sensor_data,
        )
        self.detection_pub = self.create_publisher(
            String,
            self.get_parameter("detections_topic").value,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("input_topic").value,
            self._on_live_frame,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Loading YOLO model: {MODEL_NAME}")
        self.model = YOLO(MODEL_NAME)
        self.get_logger().info(
            f"Thermal pipeline ready on {self.device}; FP16={self.use_half}"
        )

        self.display_numbers = {}
        self.next_numbers = {name: 1 for name in TEMP_RANGES}
        self.timer = self.create_timer(1.0 / 30.0, self.process_latest_frame)

    def _on_live_frame(self, message):
        if isinstance(message, CompressedImage):
            array = cv2.imdecode(
                np.frombuffer(message.data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
        else:
            try:
                array = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            except Exception as exc:
                self.get_logger().warning(f"Failed to convert live image frame: {exc}")
                return

        if array is None:
            self.get_logger().warning("Received an invalid live frame")
            return

        with self.input_lock:
            self.latest_frame = array
            self.latest_header = message.header

    def _next_number(self, object_name, track_id, fallback_index):
        if track_id is None:
            return fallback_index + 1

        key = (object_name, track_id)
        if key not in self.display_numbers:
            self.display_numbers[key] = self.next_numbers[object_name]
            self.next_numbers[object_name] += 1
        return self.display_numbers[key]

    def _encode(self, image, header):
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not ok:
            raise RuntimeError("JPEG encoding failed")

        message = CompressedImage()
        message.header = header
        message.format = "jpeg"
        message.data = encoded.tobytes()
        return message

    def process_latest_frame(self):
        with self.input_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame
            header = self.latest_header
            self.latest_frame = None

        frame = cv2.resize(
            frame,
            (FRAME_WIDTH, FRAME_HEIGHT),
            interpolation=cv2.INTER_LINEAR,
        )
        thermal, gray = make_cool_background(frame)
        marked = frame.copy()
        detection_text = []

        with torch.inference_mode():
            results = self.model.track(
                frame,
                conf=self.confidence,
                iou=self.iou,
                persist=True,
                device=self.device,
                tracker="bytetrack.yaml",
                imgsz=self.inference_size,
                half=self.use_half,
                verbose=False,
            )

        for result in results:
            if result.boxes is None:
                continue

            boxes = result.boxes
            masks = result.masks.data if result.masks is not None else None
            same_class_count = {}

            for index in range(len(boxes)):
                class_id = int(boxes.cls[index].item())
                object_name = LABELED.get(class_id) or THERMAL_ONLY.get(class_id)
                if object_name is None:
                    continue

                confidence = float(boxes.conf[index].item())
                if confidence < self.confidence:
                    continue

                same_class_count[object_name] = same_class_count.get(object_name, 0) + 1
                coords = boxes.xyxy[index].detach().cpu().numpy().astype(int).tolist()
                x1, y1, x2, y2 = coords
                x1 = max(0, min(FRAME_WIDTH - 2, x1))
                y1 = max(0, min(FRAME_HEIGHT - 2, y1))
                x2 = max(x1 + 1, min(FRAME_WIDTH - 1, x2))
                y2 = max(y1 + 1, min(FRAME_HEIGHT - 1, y2))

                track_id = None
                if boxes.id is not None:
                    track_id = int(boxes.id[index].item())
                number = self._next_number(
                    object_name,
                    track_id,
                    same_class_count[object_name] - 1,
                )

                if masks is not None:
                    mask = cv2.resize(
                        masks[index].detach().cpu().numpy().astype("float32"),
                        (FRAME_WIDTH, FRAME_HEIGHT),
                        interpolation=cv2.INTER_LINEAR,
                    )
                    temperature = get_temperature(object_name, track_id, x1, y1)
                    if object_name == "PERSON":
                        heat = human_heat(mask, temperature, gray)
                    else:
                        heat = electronic_heat(mask, temperature, object_name, gray)
                    thermal = composite_heat(thermal, heat, mask)

                cv2.rectangle(marked, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(
                    marked,
                    f"{object_name} {number}",
                    (x1, max(24, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                detection_text.append(f"{object_name}{number} {confidence:.2f}")

        draw_temperature_scale(thermal)
        draw_bottom_ui(thermal)

        try:
            self.thermal_pub.publish(self._encode(thermal, header))
            self.marked_pub.publish(self._encode(marked, header))
        except Exception as exc:
            self.get_logger().error(f"Failed to publish processed frames: {exc}")
            return

        detection_message = String()
        detection_message.data = " | ".join(detection_text) or "No supported detections"
        self.detection_pub.publish(detection_message)

        if self.show_window:
            cv2.imshow("THERMAL", thermal)
            cv2.imshow("LIVE MARKED", marked)
            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThermalNode()
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