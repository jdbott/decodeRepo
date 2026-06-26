//lot of issues btw, js ignore them all, (python) its gonna go in the limelight software



#!/usr/bin/env python3
"""
Limelight Yellow Pickleball Detector
Optimized for standard yellow pickleballs (~74mm / 2.87in diameter, perforated).
Connects to Limelight MJPEG stream, performs HSV thresholding + morphological
closing to handle holes, and publishes target data to NetworkTables.
"""

import cv2
import numpy as np
import urllib.request
import threading
import time
import json
from pathlib import Path

# NetworkTables (NT4) - install with: pip install pyntcore
        try:
import ntcore
NT_AVAILABLE = True
except ImportError:
NT_AVAILABLE = False
print("Warning: pyntcore not installed. NetworkTables disabled.")
print("  Install with: pip install pyntcore")

# ==================== CONFIGURATION ====================
LIMELIGHT_IP = "limelight.local"  # Change to your Limelight's IP if needed
STREAM_URL = f"http://{LIMELIGHT_IP}:5800/stream.mjpg"

        # Pickleball physical constants
        PICKLEBALL_DIAMETER_INCHES = 2.87  # ~74mm
        PICKLEBALL_DIAMETER_MM = 74.0

# Yellow HSV thresholds — pre-tuned for bright matte yellow pickleballs
# The holes create dark spots; we handle that with morphological closing
YELLOW_HSV_LOW = np.array([18, 120, 120])   # Lower bound (H, S, V)
YELLOW_HSV_HIGH = np.array([38, 255, 255])  # Upper bound

# Morphological operations — CLOSING is critical to fill pickleball holes
        MORPH_KERNEL_SIZE = 7       # Larger kernel to bridge hole gaps
MORPH_CLOSE_ITERATIONS = 3  # Fill the perforated holes
MORPH_OPEN_ITERATIONS = 1   # Remove small noise

# Ball detection parameters
        MIN_BALL_AREA = 300         # Minimum contour area (pixels)
MAX_BALL_AREA = 12000       # Maximum contour area
        CIRCULARITY_THRESHOLD = 0.65  # Minimum circularity (0-1)
ASPECT_RATIO_TOLERANCE = 0.3  # Max deviation from 1.0 for bounding box

# Camera / Limelight parameters
CAMERA_FOV_X = 59.6         # Limelight 2+ horizontal FOV (degrees)
CAMERA_FOV_Y = 45.7         # Limelight 2+ vertical FOV (degrees)
EXPECTED_FRAME_WIDTH = 960
EXPECTED_FRAME_HEIGHT = 720

        # Distance estimation focal length (pixels)
# CALIBRATE THIS: place ball at 36 inches, note radius in pixels,
# then: focal_length = (radius * 2 * 36) / 2.87
FOCAL_LENGTH_PIXELS = 520   # Default guess — TUNE ME!

        # NetworkTables config
NT_TABLE_NAME = "PickleballDetector"
NT_SERVER_IP = LIMELIGHT_IP  # Or your roboRIO IP

# Config file for saving tuned HSV
        CONFIG_PATH = Path("pickleball_hsv_config.json")

# ==================== GLOBAL STATE ====================
latest_frame = None
        frame_lock = threading.Lock()
running = True


class MjpegStream:
        """Async MJPEG stream reader from Limelight."""
def __init__(self, url):
self.url = url
self.stream = None
self.bytes_buffer = b""
        self._connect()

def _connect(self):
        try:
self.stream = urllib.request.urlopen(self.url, timeout=5)
print(f"[OK] Connected to Limelight stream at {self.url}")
except Exception as e:
print(f"[ERR] Failed to connect to {self.url}: {e}")
self.stream = None

def read_frame(self):
        if self.stream is None:
        self._connect()
            time.sleep(1)
            return None

        try:
self.bytes_buffer += self.stream.read(4096)
a = self.bytes_buffer.find(b"\xff\xd8")
b = self.bytes_buffer.find(b"\xff\xd9")

            if a != -1 and b != -1 and b > a:
jpg = self.bytes_buffer[a:b+2]
self.bytes_buffer = self.bytes_buffer[b+2:]
frame = cv2.imdecode(
        np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                        return frame
elif len(self.bytes_buffer) > 500000:
self.bytes_buffer = b""  # Prevent buffer overflow
            return None
except Exception as e:
print(f"[ERR] Stream error: {e}")
self.stream = None
            return None


def stream_thread(stream):
        """Background thread to continuously read frames."""
        global latest_frame, running
    while running:
frame = stream.read_frame()
        if frame is not None:
with frame_lock:
latest_frame = frame.copy()


def calculate_circularity(contour):
        """Calculate circularity: 1.0 = perfect circle."""
area = cv2.contourArea(contour)
    if area <= 0:
        return 0.0
perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0:
        return 0.0
        return (4.0 * np.pi * area) / (perimeter ** 2)


def calculate_distance(ball_radius_pixels):
        """
    Estimate distance to pickleball using known physical diameter.
    Returns distance in inches.
    """
        if ball_radius_pixels <= 0:
        return 0.0
        return (PICKLEBALL_DIAMETER_INCHES * FOCAL_LENGTH_PIXELS) / (ball_radius_pixels * 2.0)


def detect_pickelballs(frame, hsv_low, hsv_high):
        """
    Process frame to detect yellow pickleballs.
    Returns: (annotated_frame, balls_list, mask)
    """
height, width = frame.shape[:2]
frame_center_x = width // 2
        frame_center_y = height // 2

    # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create mask for yellow
        mask = cv2.inRange(hsv, hsv_low, hsv_high)

    # Morphological operations — CRITICAL for pickleballs due to holes
kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)

        # OPEN: remove small noise first
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                        iterations=MORPH_OPEN_ITERATIONS)

    # CLOSE: fill the holes inside the pickleball
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                        iterations=MORPH_CLOSE_ITERATIONS)

    # Optional: dilate slightly to ensure solid ball shape
        mask = cv2.dilate(mask, kernel, iterations=1)

    # Find contours
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

balls = []

        for cnt in contours:
area = cv2.contourArea(cnt)
        if area < MIN_BALL_AREA or area > MAX_BALL_AREA:
        continue

        # Get bounding box for aspect ratio check
        x, y, w, h = cv2.boundingRect(cnt)
aspect_ratio = min(w, h) / max(w, h) if max(w, h) > 0 else 0
        if aspect_ratio < (1.0 - ASPECT_RATIO_TOLERANCE):
        continue

        # Check circularity
circularity = calculate_circularity(cnt)
        if circularity < CIRCULARITY_THRESHOLD:
        continue

        # Get enclosing circle (best fit for ball)
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
center = (int(cx), int(cy))
radius = int(radius)

        # Calculate normalized offsets (-1 to 1)
offset_x = (cx - frame_center_x) / (width / 2.0)
offset_y = (cy - frame_center_y) / (height / 2.0)

        # Convert to degrees from camera center
angle_x = offset_x * (CAMERA_FOV_X / 2.0)
angle_y = offset_y * (CAMERA_FOV_Y / 2.0)

        # Estimate distance
distance_in = calculate_distance(radius)
distance_mm = distance_in * 25.4

ball = {
        "center": center,
        "radius": radius,
        "area": area,
        "circularity": circularity,
        "aspect_ratio": aspect_ratio,
        "offset_x": offset_x,
        "offset_y": offset_y,
        "angle_x": angle_x,      # Horizontal angle from center (degrees)
            "angle_y": angle_y,      # Vertical angle from center (degrees)
            "distance_in": distance_in,
        "distance_mm": distance_mm,
        "contour": cnt
        }
                balls.append(ball)

    # Sort by area (largest first, most likely closest/best)
    balls.sort(key=lambda b: b["area"], reverse=True)

    # Draw annotations
annotated = frame.copy()

    for i, ball in enumerate(balls):
        # Highlight best ball in green, others in orange
        color = (0, 255, 0) if i == 0 else (0, 140, 255)

        # Draw circle around ball
        cv2.circle(annotated, ball["center"], ball["radius"], color, 2)
        # Draw center dot
        cv2.circle(annotated, ball["center"], 3, (0, 0, 255), -1)

        # Label with distance and circularity
        label_y = ball["center"][1] - ball["radius"] - 10
label1 = f"{ball['distance_in']:.1f}in | {ball['distance_mm']:.0f}mm"
label2 = f"circ:{ball['circularity']:.2f} area:{ball['area']:.0f}"

        cv2.putText(annotated, label1,
                   (ball["center"][0] - 60, label_y),
cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.putText(annotated, label2,
                   (ball["center"][0] - 60, label_y + 18),
cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Draw crosshair
    cv2.line(annotated, (frame_center_x, 0), (frame_center_x, height), (255, 0, 0), 1)
        cv2.line(annotated, (0, frame_center_y), (width, frame_center_y), (255, 0, 0), 1)

        # Show mask preview in corner
        mask_preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
preview_h = height // 4
        preview_w = width // 4
mask_preview = cv2.resize(mask_preview, (preview_w, preview_h))
annotated[0:preview_h, 0:preview_w] = mask_preview
    cv2.rectangle(annotated, (0, 0), (preview_w, preview_h), (0, 255, 0), 1)

        return annotated, balls, mask


def load_config():
        """Load saved HSV config if it exists."""
        if CONFIG_PATH.exists():
        try:
with open(CONFIG_PATH) as f:
cfg = json.load(f)
low = np.array(cfg.get("low", [18, 120, 120]))
high = np.array(cfg.get("high", [38, 255, 255]))
print(f"[OK] Loaded HSV config from {CONFIG_PATH}")
            return low, high
except Exception as e:
print(f"[WARN] Could not load config: {e}")
    return None, None


def save_config(hsv_low, hsv_high):
        """Save HSV config to JSON file."""
cfg = {
        "low": hsv_low.tolist(),
        "high": hsv_high.tolist(),
        "note": "Yellow pickleball HSV thresholds"
                }
                try:
with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
print(f"[OK] Saved HSV config to {CONFIG_PATH}")
except Exception as e:
print(f"[ERR] Could not save config: {e}")


def main():
        global running, latest_frame, FOCAL_LENGTH_PIXELS

print("=" * 55)
print("  Limelight Yellow Pickleball Detector")
print("  Target: 74mm / 2.87in diameter perforated yellow ball")
print("=" * 55)
print(f"  Stream: {STREAM_URL}")
print("  Controls:")
print("    'q'  — Quit")
print("    's'  — Save current HSV values to pickleball_hsv_config.json")
print("    'c'  — Calibrate focal length (place ball at 36in, press 'c')")
print("    'r'  — Reset HSV to defaults")
print("  Trackbars — Tune yellow detection in real-time")
print("=" * 55)

    # Initialize NetworkTables
nt_inst = None
        pub_table = None
    if NT_AVAILABLE:
nt_inst = ntcore.NetworkTableInstance.getDefault()
        nt_inst.startClient4("python-pickleball-detector")
        nt_inst.setServer(NT_SERVER_IP)
pub_table = nt_inst.getTable(NT_TABLE_NAME)
print("[OK] NetworkTables client started")
    else:
print("[INFO] NetworkTables not available (pip install pyntcore)")

    # Start stream
stream = MjpegStream(STREAM_URL)
stream_thread_obj = threading.Thread(target=stream_thread, args=(stream,), daemon=True)
        stream_thread_obj.start()

    # Wait for first frame
print("[...] Waiting for video stream...")
timeout = 0
        while latest_frame is None and timeout < 15:
        time.sleep(0.5)
timeout += 0.5
        if latest_frame is None:
print("[ERR] Could not get frames from Limelight! Check IP/network.")
        return

                # Load saved config or use defaults
saved_low, saved_high = load_config()
    if saved_low is not None and saved_high is not None:
hsv_low = saved_low
        hsv_high = saved_high
    else:
hsv_low = YELLOW_HSV_LOW.copy()
hsv_high = YELLOW_HSV_HIGH.copy()

    # Create windows and trackbars
    cv2.namedWindow("Pickleball Detector", cv2.WINDOW_NORMAL)
    cv2.namedWindow("HSV Tuning", cv2.WINDOW_NORMAL)

def nothing(x):
pass

    cv2.createTrackbar("H Low",  "HSV Tuning", int(hsv_low[0]), 179, nothing)
        cv2.createTrackbar("H High", "HSV Tuning", int(hsv_high[0]), 179, nothing)
        cv2.createTrackbar("S Low",  "HSV Tuning", int(hsv_low[1]), 255, nothing)
        cv2.createTrackbar("S High", "HSV Tuning", int(hsv_high[1]), 255, nothing)
        cv2.createTrackbar("V Low",  "HSV Tuning", int(hsv_low[2]), 255, nothing)
        cv2.createTrackbar("V High", "HSV Tuning", int(hsv_high[2]), 255, nothing)
        cv2.createTrackbar("Min Area", "HSV Tuning", MIN_BALL_AREA, 2000, nothing)
    cv2.createTrackbar("Circ %", "HSV Tuning", int(CIRCULARITY_THRESHOLD * 100), 100, nothing)
        cv2.createTrackbar("Close Iter", "HSV Tuning", MORPH_CLOSE_ITERATIONS, 10, nothing)
    cv2.createTrackbar("Kern Size", "HSV Tuning", MORPH_KERNEL_SIZE, 15, nothing)

calibration_mode = False

    try:
            while True:
with frame_lock:
        if latest_frame is None:
        continue
frame = latest_frame.copy()

            # Get trackbar values
        h_low = cv2.getTrackbarPos("H Low", "HSV Tuning")
h_high = cv2.getTrackbarPos("H High", "HSV Tuning")
s_low = cv2.getTrackbarPos("S Low", "HSV Tuning")
s_high = cv2.getTrackbarPos("S High", "HSV Tuning")
v_low = cv2.getTrackbarPos("V Low", "HSV Tuning")
v_high = cv2.getTrackbarPos("V High", "HSV Tuning")
min_area = cv2.getTrackbarPos("Min Area", "HSV Tuning")
circ = cv2.getTrackbarPos("Circ %", "HSV Tuning") / 100.0
close_iter = cv2.getTrackbarPos("Close Iter", "HSV Tuning")
kern_size = cv2.getTrackbarPos("Kern Size", "HSV Tuning")

hsv_low = np.array([h_low, s_low, v_low])
hsv_high = np.array([h_high, s_high, v_high])

            # Temporarily override globals for detection
        global MIN_BALL_AREA, CIRCULARITY_THRESHOLD, MORPH_CLOSE_ITERATIONS, MORPH_KERNEL_SIZE
MIN_BALL_AREA = min_area
        CIRCULARITY_THRESHOLD = circ
MORPH_CLOSE_ITERATIONS = close_iter
        MORPH_KERNEL_SIZE = max(1, kern_size)

            # Detect pickleballs
annotated, balls, mask = detect_pickelballs(frame, hsv_low, hsv_high)

            # Overlay HUD
hud_lines = [
f"Pickleballs: {len(balls)}",
f"Res: {frame.shape[1]}x{frame.shape[0]}",
f"FOCAL: {FOCAL_LENGTH_PIXELS}px",
        ]
        if balls:
best = balls[0]
        hud_lines.extend([
                         f"Angle: ({best['angle_x']:.1f}°, {best['angle_y']:.1f}°)",
                         f"Dist: {best['distance_in']:.1f}in / {best['distance_mm']:.0f}mm",
                         f"Area: {best['area']:.0f}  Circ: {best['circularity']:.2f}",
                ])
            if calibration_mode:
        hud_lines.append(">>> CALIBRATION MODE: Place ball at 36in, press 'c'")

y_offset = 30
        for line in hud_lines:
        cv2.putText(annotated, line, (10, y_offset),
cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
y_offset += 22

        # Publish to NetworkTables (FRC-compatible naming)
            if pub_table is not None:
        pub_table.putBoolean("tv", len(balls) > 0)  # Target valid
                pub_table.putNumber("tx", balls[0]["angle_x"] if balls else 0.0)
                pub_table.putNumber("ty", balls[0]["angle_y"] if balls else 0.0)
                pub_table.putNumber("ta", balls[0]["area"] / (frame.shape[0] * frame.shape[1]) if balls else 0.0)
        pub_table.putNumber("ball_count", len(balls))
        if balls:
        pub_table.putNumber("distance_in", balls[0]["distance_in"])
                    pub_table.putNumber("distance_mm", balls[0]["distance_mm"])
                    pub_table.putNumber("radius_px", balls[0]["radius"])

            # Show frames
            cv2.imshow("Pickleball Detector", annotated)
            cv2.imshow("HSV Tuning", cv2.resize(mask, (400, 300)))

key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
        break
elif key == ord('s'):
save_config(hsv_low, hsv_high)
print(f"  HSV Low:  {hsv_low.tolist()}")
print(f"  HSV High: {hsv_high.tolist()}")
elif key == ord('c'):
        if balls:
        # Calibrate: assume ball is at 36 inches
        CALIB_DISTANCE = 36.0
best = balls[0]
new_focal = (best["radius"] * 2 * CALIB_DISTANCE) / PICKLEBALL_DIAMETER_INCHES
print(f"[CALIB] Old focal: {FOCAL_LENGTH_PIXELS:.1f}")
print(f"[CALIB] New focal: {new_focal:.1f} (based on r={best['radius']}px at 36in)")
FOCAL_LENGTH_PIXELS = new_focal
                else:
print("[CALIB] No ball detected! Cannot calibrate.")
elif key == ord('r'):
        cv2.setTrackbarPos("H Low", "HSV Tuning", 18)
                cv2.setTrackbarPos("H High", "HSV Tuning", 38)
                cv2.setTrackbarPos("S Low", "HSV Tuning", 120)
                cv2.setTrackbarPos("S High", "HSV Tuning", 255)
                cv2.setTrackbarPos("V Low", "HSV Tuning", 120)
                cv2.setTrackbarPos("V High", "HSV Tuning", 255)
print("[OK] Reset HSV to defaults")

except KeyboardInterrupt:
print("\n[OK] Interrupted by user")
    finally:
running = False
        cv2.destroyAllWindows()
print("[OK] Shutdown complete")
print(f"\nFinal focal length: {FOCAL_LENGTH_PIXELS:.1f}px")
print(f"Final HSV Low:  {hsv_low.tolist()}")
print(f"Final HSV High: {hsv_high.tolist()}")


if __name__ == "__main__":
main()