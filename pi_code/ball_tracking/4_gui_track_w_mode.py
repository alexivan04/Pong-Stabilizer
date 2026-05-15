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
PACKET_TYPE_PATTERN_CHANGE = 4  # New packet type for receiving patterns

# Mapping for the Enum
PATTERN_NAMES = {
    0: "CENTER",
    1: "CIRCLE",
    2: "STAR",
    3: "FIGURE 8",
    4: "CYCLE ALL"
}
current_pattern_text = "CENTER" # Default starting text

SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0) # timeout=0 for non-blocking
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

def receive_packets():
    """Checks serial buffer for incoming pattern updates"""
    global current_pattern_text
    # We look for a packet: [START][TYPE][SIZE][PAYLOAD][CSUM][END]
    # For Pattern Change, payload is usually 1 byte (the enum index)
    if ser.in_waiting >= 6: 
        if ser.read(1)[0] == START_BYTE:
            p_type = ser.read(1)[0]
            p_size = ser.read(1)[0]
            payload = ser.read(p_size)
            checksum = ser.read(1)[0]
            end = ser.read(1)[0]

            if end == END_BYTE and p_type == PACKET_TYPE_PATTERN_CHANGE:
                pattern_idx = payload[0]
                current_pattern_text = PATTERN_NAMES.get(pattern_idx, "UNKNOWN")

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
picam2.set_controls({"FrameDurationLimits": (int(1e6/60), int(1e6/60)), "ExposureTime": 3000, "AnalogueGain": 1.0, "AeEnable": False, "AwbEnable": False})
picam2.start()
kernel = np.ones((3,3), np.uint8)

send_packet(PACKET_TYPE_START, b'')
isStarted = False

# --- TRACKING VARIABLES ---
prev_time = time.time()
prev_smooth_x, prev_smooth_y, prev_smooth_z = 0.0, 0.0, 0.0
vx, vy, vz = 0.0, 0.0, 0.0

POS_ALPHA = 0.6       
VELOCITY_ALPHA = 0.15 
DEADBAND = 1.5        
first_detection = True 

try:
    while True:
        # 1. Check for incoming Serial Data (Pattern Updates)
        receive_packets()

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
                
                # Draw ball tracker
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

        # --- CALCULATE VELOCITY ---
        curr_time = time.time()
        dt = curr_time - prev_time
        if dt <= 0: dt = 0.016 

        if is_found:
            if first_detection:
                prev_smooth_x, prev_smooth_y, prev_smooth_z = X_real, Y_real, Z_depth
                first_detection = False

            smooth_x = (POS_ALPHA * X_real) + ((1.0 - POS_ALPHA) * prev_smooth_x)
            smooth_y = (POS_ALPHA * Y_real) + ((1.0 - POS_ALPHA) * prev_smooth_y)
            smooth_z = (POS_ALPHA * Z_depth) + ((1.0 - POS_ALPHA) * prev_smooth_z)

            dx, dy, dz = smooth_x - prev_smooth_x, smooth_y - prev_smooth_y, smooth_z - prev_smooth_z
            if abs(dx) < DEADBAND: dx = 0.0
            if abs(dy) < DEADBAND: dy = 0.0
            if abs(dz) < DEADBAND: dz = 0.0

            vx = (VELOCITY_ALPHA * (dx/dt)) + ((1.0 - VELOCITY_ALPHA) * vx)
            vy = (VELOCITY_ALPHA * (dy/dt)) + ((1.0 - VELOCITY_ALPHA) * vy)
            vz = (VELOCITY_ALPHA * (dz/dt)) + ((1.0 - VELOCITY_ALPHA) * vz)
            prev_smooth_x, prev_smooth_y, prev_smooth_z = smooth_x, smooth_y, smooth_z
        else:
            vx, vy, vz = vx*0.8, vy*0.8, vz*0.8
            first_detection = True

        prev_time = curr_time

        # --- ONSCREEN DISPLAY ---
        # Draw the current pattern text in the top left corner
        cv2.putText(frame, f"MODE: {current_pattern_text}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        cv2.imshow("Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # --- SEND DATA TO TEENSY ---
        if is_found and not isStarted:
            send_packet(PACKET_TYPE_START, b'')
            isStarted = True
            
        if isStarted:
            ball_payload = struct.pack('<Bffffff', is_found, float(X_real), float(Y_real), float(Z_depth), float(vx), float(vy), float(vz))
            send_packet(PACKET_TYPE_BALL_DATA, ball_payload)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    picam2.stop()
    if 'ser' in locals() and ser.is_open:
        ser.close()
    cv2.destroyAllWindows()
