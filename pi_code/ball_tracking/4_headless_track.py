import cv2
from picamera2 import Picamera2
import numpy as np
import time
import serial
import struct
import os

# --- UART COMMUNICATION PROTOCOL ---
START_BYTE = 0xAA
END_BYTE = 0xBB
PACKET_TYPE_BALL_DATA = 1

SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

# Open Serial Port
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print(f"UART Initialized on {SERIAL_PORT} at {BAUD_RATE} bps.")
except serial.SerialException as e:
    print(f"Serial Port Error: {e}")
    exit()

def send_packet(packet_type, payload):
    """Constructs and sends a complete packet over UART."""
    payload_size = len(payload)
    
    # Calculate checksum (XOR of type, size, and all payload bytes)
    checksum = packet_type ^ payload_size
    for byte in payload:
        checksum ^= byte
        
    # Pack header (< = Little-endian, B = unsigned char)
    header = struct.pack('<BBB', START_BYTE, packet_type, payload_size)
    footer = struct.pack('<BB', checksum, END_BYTE)
    
    # Send complete packet
    ser.write(header + payload + footer)

def load_hsv(filename="hsv_values.txt"):
    try:
        with open(filename, 'r') as f:
            lines =[line.strip() for line in f if line.strip()]
            lower = np.array([int(x) for x in lines[0].split(',')])
            upper = np.array([int(x) for x in lines[1].split(',')])
            return lower, upper
    except Exception as e:
        print(f"Warning: Could not read {filename}. Using defaults. ({e})")
        return np.array([106, 131, 57]), np.array([123, 255, 165])

# --- USER CONFIGURATION ---
LOWER_HSV, UPPER_HSV = load_hsv()

REAL_BALL_DIAMETER_MM = 40.0
# Set to 0 for "Distance from Camera". Set to e.g. 2400 for "Height from Floor".
CAMERA_HEIGHT_FROM_FLOOR_MM = 0 

WIDTH, HEIGHT = 640, 480
FRAMERATE = 60
SHUTTER = 10000
GAIN = 2.0
# --------------------------

# -------- LOAD FISHEYE CALIBRATION --------
try:
    with np.load('cam_calib_fisheye.npz') as data:
        K_fisheye = data['K']
        D_fisheye = data['D']
except FileNotFoundError:
    print("FATAL ERROR: 'cam_calib_fisheye.npz' not found.")
    print("Please run camera_calib/2_calibrate_calc.py first!")
    exit()

# Generate the "New" Camera Matrix (Linear/Pinhole model)
balance = 1.0 # Keep this 1.0 to match the visual script exactly
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
# -------------------------------------------

# --- picamera2 setup ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "BGR888"}
)
picam2.configure(config)
picam2.set_controls({
    "FrameDurationLimits": (int(1e6/FRAMERATE), int(1e6/FRAMERATE)),
    "ExposureTime": SHUTTER,
    "AnalogueGain": GAIN,
    "AeEnable": False,
    "AwbEnable": False,
})

kernel = np.ones((3,3), np.uint8)

print("Starting Camera...")
picam2.start()
time.sleep(1) # Allow camera to warm up
print("--- Headless Tracker & UART Streaming Started ---")
print("Using 'Undistort First + Correction' Method.")
print("Press Ctrl+C to quit.")

try:
    while True:
        raw_frame = picam2.capture_array()

        # --- STEP A: UNDISTORT FULL IMAGE ---
        frame = cv2.remap(raw_frame, map1, map2, cv2.INTER_LINEAR)

        # --- STEP B: VISION PROCESSING ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Default values if ball is not found
        is_found = 0
        X_real = 0.0
        Y_real = 0.0
        Z_depth = 0.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 5:
                is_found = 1
                
                # --- STEP C: CALCULATE POSITION ---
                # 1. Base Pinhole Depth
                pixel_diameter = radius * 2
                Z_pinhole = (fx * REAL_BALL_DIAMETER_MM) / pixel_diameter
                
                # 2. Angle Correction (fixes the Z-Curve)
                dist_from_center = np.sqrt((x - cx)**2 + (y - cy)**2)
                correction_factor = np.sqrt(1 + (dist_from_center / fx)**2)
                Z_final_cam = Z_pinhole * correction_factor

                # 3. Real X and Y
                X_real = ((x - cx) * Z_final_cam) / fx
                Y_real = ((y - cy) * Z_final_cam) / fy
                
                # 4. Floor Height Logic
                if CAMERA_HEIGHT_FROM_FLOOR_MM > 0:
                    Z_depth = CAMERA_HEIGHT_FROM_FLOOR_MM - Z_final_cam
                else:
                    Z_depth = Z_final_cam

        # --- SEND DATA TO TEENSY OVER UART ---
        # Format payload: < (little endian), B (1 byte unsigned char), fff (three 4-byte floats)
        payload = struct.pack('<Bfff', is_found, float(X_real), float(Y_real), float(Z_depth))
        send_packet(PACKET_TYPE_BALL_DATA, payload)
        # --------------------------------------

        # Optional: Print to console
        if is_found:
             print(f"Tracking -> X:{X_real:.1f} Y:{Y_real:.1f} Z:{Z_depth:.1f} mm   ", end='\r')
        else:
             print("Searching...                                  ", end='\r')

except KeyboardInterrupt:
    print("\n\nProgram stopped by user (Ctrl+C).")

finally:
    picam2.stop()
    if 'ser' in locals() and ser.is_open:
        ser.close()
    print("Shutdown complete.")
