#!/usr/bin/env python3
import cv2
from picamera2 import Picamera2
import numpy as np
import time
import serial
import struct
import os

# --- UART CONFIG ---
START_BYTE = 0xAA
END_BYTE = 0xBB
PACKET_TYPE_BALL_DATA = 1
PACKET_TYPE_PID_UPDATE = 2
PACKET_TYPE_START = 3

SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
except serial.SerialException as e:
    print("Could not open serial port.")
    exit()

def send_packet(packet_type, payload):
    payload_size = len(payload)
    checksum = packet_type ^ payload_size
    for byte in payload:
        checksum ^= byte
    header = struct.pack('<BBB', START_BYTE, packet_type, payload_size)
    footer = struct.pack('<BB', checksum, END_BYTE)
    ser.write(header + payload + footer)

def nothing(x): pass

# --- CAMERA CONFIG ---
def load_hsv(filename="hsv_values.txt"):
    try:
        with open(filename, 'r') as f:
            lines =[line.strip() for line in f if line.strip()]
            return np.array([int(x) for x in lines[0].split(',')]), np.array([int(x) for x in lines[1].split(',')])
    except:
        return np.array([106, 131, 57]), np.array([123, 255, 165])

LOWER_HSV, UPPER_HSV = load_hsv()
REAL_BALL_DIAMETER_MM = 40.0
WIDTH, HEIGHT = 640, 480

try:
    with np.load('cam_calib_fisheye.npz') as data:
        K_fisheye, D_fisheye = data['K'], data['D']
except:
    print("No calibration data found.")
    exit()

new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K_fisheye, D_fisheye, (WIDTH, HEIGHT), np.eye(3), balance=1.0)
fx, fy = new_K[0, 0], new_K[1, 1]
cx, cy = new_K[0, 2], new_K[1, 2]
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K_fisheye, D_fisheye, np.eye(3), new_K, (WIDTH, HEIGHT), cv2.CV_16SC2)

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (WIDTH, HEIGHT), "format": "BGR888"})
picam2.configure(config)
picam2.set_controls({"FrameDurationLimits": (int(1e6/60), int(1e6/60)), "ExposureTime": 10000, "AnalogueGain": 2.0, "AeEnable": False, "AwbEnable": False})
picam2.start()
kernel = np.ones((3,3), np.uint8)

send_packet(PACKET_TYPE_START, b'')
isStarted = False

# --- TRACKING VARIABLES ---
prev_time = time.time()
prev_smooth_x, prev_smooth_y, prev_smooth_z = 0.0, 0.0, 0.0
vx, vy, vz = 0.0, 0.0, 0.0

POS_ALPHA = 0.6       # Smooths out raw camera pixel jitter (lower = smoother but slightly delayed)
VELOCITY_ALPHA = 0.15 # Smooths the resulting velocity 
DEADBAND = 1.5        # mm threshold: Ignore movements smaller than this to prevent idle jitter
first_detection = True # Flag to prevent huge initial velocity spikes

try:
    while True:
        raw_frame = picam2.capture_array()
        frame = cv2.remap(raw_frame, map1, map2, cv2.INTER_LINEAR)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        is_found, X_real, Y_real, Z_depth = 0, 0.0, 0.0, 0.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 5:
                is_found = 1
                Z_pinhole = (fx * REAL_BALL_DIAMETER_MM) / (radius * 2)
                dist_from_center = np.sqrt((x - cx)**2 + (y - cy)**2)
                Z_depth = Z_pinhole * np.sqrt(1 + (dist_from_center / fx)**2)
                X_real = ((x - cx) * Z_depth) / fx
                Y_real = ((y - cy) * Z_depth) / fy

        # --- CALCULATE VELOCITY ---
        curr_time = time.time()
        dt = curr_time - prev_time
        if dt <= 0:
            dt = 0.016 

        if is_found:
            # Handle the very first frame the ball is seen
            if first_detection:
                prev_smooth_x, prev_smooth_y, prev_smooth_z = X_real, Y_real, Z_depth
                first_detection = False

            # 1. Smooth the raw position to filter out camera noise
            smooth_x = (POS_ALPHA * X_real) + ((1.0 - POS_ALPHA) * prev_smooth_x)
            smooth_y = (POS_ALPHA * Y_real) + ((1.0 - POS_ALPHA) * prev_smooth_y)
            smooth_z = (POS_ALPHA * Z_depth) + ((1.0 - POS_ALPHA) * prev_smooth_z)

            # 2. Find the change in distance
            dx = smooth_x - prev_smooth_x
            dy = smooth_y - prev_smooth_y
            dz = smooth_z - prev_smooth_z

            # 3. Apply Deadband: If the change is tiny, treat it as ZERO
            if abs(dx) < DEADBAND: dx = 0.0
            if abs(dy) < DEADBAND: dy = 0.0
            if abs(dz) < DEADBAND: dz = 0.0

            # 4. Calculate raw velocity
            raw_vx = dx / dt
            raw_vy = dy / dt
            raw_vz = dz / dt

            # 5. Apply Low-Pass Filter to velocity
            vx = (VELOCITY_ALPHA * raw_vx) + ((1.0 - VELOCITY_ALPHA) * vx)
            vy = (VELOCITY_ALPHA * raw_vy) + ((1.0 - VELOCITY_ALPHA) * vy)
            vz = (VELOCITY_ALPHA * raw_vz) + ((1.0 - VELOCITY_ALPHA) * vz)

            # Store current position for next frame
            prev_smooth_x, prev_smooth_y, prev_smooth_z = smooth_x, smooth_y, smooth_z
        else:
            # Decay velocity to zero if ball is lost, reset first detection flag
            vx *= 0.8
            vy *= 0.8
            vz *= 0.8
            first_detection = True

        prev_time = curr_time

        # --- SEND DATA TO TEENSY ---
        if is_found and not isStarted:
            send_packet(PACKET_TYPE_START, b'')
            isStarted = True
            
        if isStarted:
            # Packing 1 byte (B) and 6 floats (f) = 25 bytes total payload
            ball_payload = struct.pack('<Bffffff', is_found, float(X_real), float(Y_real), float(Z_depth), float(vx), float(vy), float(vz))
            send_packet(PACKET_TYPE_BALL_DATA, ball_payload)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    picam2.stop()
    if 'ser' in locals() and ser.is_open:
        ser.close()
    cv2.destroyAllWindows()