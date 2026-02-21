import cv2
from picamera2 import Picamera2
import numpy as np
import time

# --- USER CONFIGURATION ---
# Values for bad lighting
LOWER_HSV = np.array([106, 131, 57])
UPPER_HSV = np.array([123, 255, 165])

# Values for good lighting
# LOWER_HSV = np.array([91, 164, 99])
# UPPER_HSV = np.array([123, 255, 165])

REAL_BALL_DIAMETER_MM = 40.0
WIDTH, HEIGHT = 640, 480
FRAMERATE = 60
SHUTTER = 5000
GAIN = 1.0
# --------------------------

# Load Calibration
try:
    with np.load('cam_calib.npz') as data:
        mtx, dist = data['mtx'], data['dist']
        fx, fy = mtx[0, 0], mtx[1, 1]
        cx, cy = mtx[0, 2], mtx[1, 2]
except FileNotFoundError:
    print("FATAL ERROR: Calibration file 'cam_calib.npz' not found.")
    print("Please run the calibration scripts first.")
    exit()

# --- picamera2 setup ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "BGR888"} # Capture BGR for OpenCV
)
picam2.configure(config)
picam2.set_controls({
    "FrameDurationLimits": (int(1e6/FRAMERATE), int(1e6/FRAMERATE)),
    "ExposureTime": SHUTTER,
    "AnalogueGain": GAIN,
    "AeEnable": False,
    "AwbEnable": False,
})

# pre-allocate memory for morphology
kernel = np.ones((3,3), np.uint8)

print("--- Headless Ball Tracker Initialized ---")
print(f"Using HSV range: {LOWER_HSV} to {UPPER_HSV}")
print("Starting camera stream...")
picam2.start()
time.sleep(1) # Allow camera to warm up
print("Tracking started. Press Ctrl+C to stop.")

try:
    while True:
        # frame is already in BGR format
        frame = picam2.capture_array()

        # Core processing pipeline
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Assume the largest contour is the ball
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # Filter out small, noisy detections
            if radius > 5:
                # Undistort only the center point of the ball for maximum speed
                point_distorted = np.array([[[x, y]]], dtype=np.float64)
                point_undistorted = cv2.undistortPoints(point_distorted, mtx, dist, P=mtx)
                ux, uy = point_undistorted[0][0]

                # Calculate the 3D position in millimeters
                pixel_diameter = radius * 2
                Z_depth = (fx * REAL_BALL_DIAMETER_MM) / pixel_diameter
                X_real = ((ux - cx) * Z_depth) / fx
                Y_real = ((uy - cy) * Z_depth) / fy

                # OUTPUT DATA TO CONSOLE
                print(f"BALL_FOUND, X={X_real:.1f}, Y={Y_real:.1f}, Z={Z_depth:.1f}")

            else:
                # Largest contour was too small to be the ball
                print("BALL_NOT_FOUND, reason=contour_too_small")
        else:
            # No contours found at all
            print("BALL_NOT_FOUND, reason=no_contours")

except KeyboardInterrupt:
    # This block runs when you press Ctrl+C
    print("\nProgram stopped by user.")

finally:
    # This block runs no matter how the try block exits
    print("Shutting down camera.")
    picam2.stop()
