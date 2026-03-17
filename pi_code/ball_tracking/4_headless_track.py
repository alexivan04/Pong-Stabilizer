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

SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
except serial.SerialException as e:
    exit()

def send_packet(packet_type, payload):
    payload_size = len(payload)
    checksum = packet_type ^ payload_size
    for byte in payload:
        checksum ^= byte
    header = struct.pack('<BBB', START_BYTE, packet_type, payload_size)
    footer = struct.pack('<BB', checksum, END_BYTE)
    ser.write(header + payload + footer)

# --- GUI SETUP FOR PID ---
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

try:
    while True:
        # --- 2. HANDLE BALL TRACKING ---
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

        # Send Ball Data Packet
        ball_payload = struct.pack('<Bfff', is_found, float(X_real), float(Y_real), float(Z_depth))
        send_packet(PACKET_TYPE_BALL_DATA, ball_payload)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    picam2.stop()
    if 'ser' in locals() and ser.is_open:
        ser.close()
    cv2.destroyAllWindows()
