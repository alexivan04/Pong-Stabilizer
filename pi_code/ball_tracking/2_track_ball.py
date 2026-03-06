import cv2
from picamera2 import Picamera2
import numpy as np
import time
import os

# --- 1. CONFIGURATION ---
REAL_BALL_DIAMETER_MM = 40.0
# Set to 0 for "Distance from Camera". Set to e.g. 2400 for "Height from Floor".
CAMERA_HEIGHT_FROM_FLOOR_MM = 0 

WIDTH, HEIGHT = 640, 480
FRAMERATE = 60
SHUTTER = 10000
GAIN = 2.0

# --- 2. LOAD CALIBRATION ---
try:
    with np.load('cam_calib_fisheye.npz') as data:
        K_fisheye = data['K']
        D_fisheye = data['D']
except FileNotFoundError:
    print("Error: cam_calib_fisheye.npz not found.")
    exit()

# --- 3. PRE-CALCULATE UNDISTORT MAPS ---
# balance=1.0 keeps ALL pixels (shows curved black borders).
# balance=0.0 crops to the center (removes borders but loses FOV).
balance = 1.0 

# Generate the "New" Camera Matrix (Linear/Pinhole model)
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    K_fisheye, D_fisheye, (WIDTH, HEIGHT), np.eye(3), balance=balance
)
# Extract the new Focal Lengths and Optical Center from this matrix
fx, fy = new_K[0, 0], new_K[1, 1]
cx, cy = new_K[0, 2], new_K[1, 2]

# Generate the Lookup Tables for Remapping
map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K_fisheye, D_fisheye, np.eye(3), new_K, (WIDTH, HEIGHT), cv2.CV_16SC2
)

# --- 4. HSV UTILS ---
def load_hsv(filename="hsv_values.txt"):
    try:
        with open(filename, 'r') as f:
            lines = [line.strip() for line in f if line.strip()]
            l = np.array([int(x) for x in lines[0].split(',')])
            u = np.array([int(x) for x in lines[1].split(',')])
            return l, u
    except:
        return np.array([106, 131, 57]), np.array([123, 255, 165])

LOWER_HSV, UPPER_HSV = load_hsv()

# --- 5. START CAMERA ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (WIDTH, HEIGHT), "format": "BGR888"})
picam2.configure(config)
picam2.set_controls({
    "FrameDurationLimits": (int(1e6/FRAMERATE), int(1e6/FRAMERATE)),
    "ExposureTime": SHUTTER, "AnalogueGain": GAIN,
    "AeEnable": False, "AwbEnable": False,
})
picam2.start()

kernel = np.ones((3,3), np.uint8)

print("--- VISUAL TRACKER (Undistort First) ---")
print(f"Focal Length: {fx:.1f} px")
print("Press 'q' to quit.")

while True:
    raw_frame = picam2.capture_array()

    # --- STEP A: UNDISTORT THE ENTIRE IMAGE ---
    # This creates a flat, linear image. Straight lines will look straight.
    # We do detection ON THIS IMAGE.
    frame = cv2.remap(raw_frame, map1, map2, cv2.INTER_LINEAR)

    # --- STEP B: DETECT BALL ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    debug_text = "Searching..."
    X_real, Y_real, Z_real = 0.0, 0.0, 0.0

    if contours:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 5:
            # --- STEP C: CALCULATE POSITION ---
            
            # 1. Simple Pinhole Z (The "Old Formula")
            #    Z = (Focal_Length * Real_Diameter) / Pixel_Diameter
            pixel_diameter = radius * 2
            Z_pinhole = (fx * REAL_BALL_DIAMETER_MM) / pixel_diameter
            
            # 2. ANGLE CORRECTION (The Fix for the Z-Curve)
            #    In a flat image, objects at the edge look stretched/bigger.
            #    Bigger pixels = Smaller Calculated Z.
            #    We fix this by dividing by cosine of the angle.
            
            # Distance from center of image (in pixels)
            dist_from_center = np.sqrt((x - cx)**2 + (y - cy)**2)
            
            # Calculate angle of the object from the center axis
            # tan(theta) = dist / focal_length
            theta = np.arctan(dist_from_center / fx)
            
            # Apply correction: Z_corrected = Z_pinhole / cos(theta)
            # (Using 1/cos is same as multiplying by sqrt(1 + tan^2))
            correction_factor = np.sqrt(1 + (dist_from_center / fx)**2)
            Z_final_cam = Z_pinhole * correction_factor

            # 3. Calculate X and Y
            X_real = ((x - cx) * Z_final_cam) / fx
            Y_real = ((y - cy) * Z_final_cam) / fy

            # 4. Floor Height Logic
            if CAMERA_HEIGHT_FROM_FLOOR_MM > 0:
                Z_display = CAMERA_HEIGHT_FROM_FLOOR_MM - Z_final_cam
            else:
                Z_display = Z_final_cam

            debug_text = f"X:{int(X_real)} Y:{int(Y_real)} Z:{int(Z_display)}mm"

            # Draw Visuals
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)

    # UI Overlay
    cv2.rectangle(frame, (0, 0), (640, 50), (0,0,0), -1)
    cv2.putText(frame, debug_text, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

    cv2.imshow("Undistorted Tracker", frame)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
