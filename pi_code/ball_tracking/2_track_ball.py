import cv2
from picamera2 import Picamera2
import numpy as np
import time

# --- USER CONFIGURATION ---
# Paste the values you found from the tuning script here
# Values for bad lighting
LOWER_HSV = np.array([106, 131, 57])
UPPER_HSV = np.array([123, 255, 165])

# Values for good lighting
# LOWER_HSV = np.array([91, 164, 99])
# UPPER_HSV = np.array([123, 255, 165])

REAL_BALL_DIAMETER_MM = 40.0
WIDTH, HEIGHT = 640, 480
FRAMERATE = 60
SHUTTER = 10000
GAIN = 2.0
# --------------------------

# Load Calibration
try:
    with np.load('cam_calib.npz') as data:
        mtx, dist = data['mtx'], data['dist']
        fx, fy = mtx[0, 0], mtx[1, 1]
        cx, cy = mtx[0, 2], mtx[1, 2]
except FileNotFoundError:
    print("Error: Calibration file 'cam_calib.npz' not found. Run calibration scripts first.")
    exit()

# --- picamera2 setup ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "BGR888"} # Capture BGR
)
picam2.configure(config)
picam2.set_controls({
    "FrameDurationLimits": (int(1e6/FRAMERATE), int(1e6/FRAMERATE)),
    "ExposureTime": SHUTTER,
    "AnalogueGain": GAIN,
    "AeEnable": False,
    "AwbEnable": False,
})
picam2.start()

kernel = np.ones((3,3), np.uint8)
prev_time = 0

while True:
    # frame is already in BGR format
    frame = picam2.capture_array()

    # PROCESSING (all done in BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius > 5:
            point_distorted = np.array([[[x, y]]], dtype=np.float64)
            point_undistorted = cv2.undistortPoints(point_distorted, mtx, dist, P=mtx)
            ux, uy = point_undistorted[0][0]

            pixel_diameter = radius * 2
            Z_depth = (fx * REAL_BALL_DIAMETER_MM) / pixel_diameter
            X_real = ((ux - cx) * Z_depth) / fx
            Y_real = ((uy - cy) * Z_depth) / fy

            # DRAWING (done on the BGR frame)
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
            text = f"X:{int(X_real)} Y:{int(Y_real)} Z:{int(Z_depth)} mm"
            cv2.putText(frame, text, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # FPS calculation
    current_time = time.time()
    fps = 1 / (current_time - prev_time) if prev_time else 0
    prev_time = current_time
    cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # --- DISPLAY CORRECTION ---
    # Convert BGR to RGB ONLY for the final display
    display_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.imshow("Fast Tracker", display_frame)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
