#!/usr/bin/env python3
import cv2
import numpy as np
import time
import serial
import struct
import signal

from ball_detector import create_tracker, load_undistort, start_camera

def _on_sigterm(signum, frame):
    raise KeyboardInterrupt
signal.signal(signal.SIGTERM, _on_sigterm)

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

# --- CAMERA + TRACKER (see ball_detector.py: DETECTOR_MODE picks the method) ---
map1, map2, new_K = load_undistort()
picam2  = start_camera()
tracker = create_tracker(new_K)

send_packet(PACKET_TYPE_START, b'')
isStarted = False

try:
    while True:
        raw_frame = picam2.capture_array()
        frame = cv2.remap(raw_frame, map1, map2, cv2.INTER_LINEAR)

        # position (mm) and velocity (mm/s)
        ball = tracker.update(frame)
        is_found = 1 if ball.found else 0

        # --- SEND DATA TO TEENSY ---
        if is_found and not isStarted:
            send_packet(PACKET_TYPE_START, b'')
            isStarted = True
            
        if isStarted:
            # Packing 1 byte (B) and 6 floats (f) = 25 bytes total payload
            ball_payload = struct.pack('<Bffffff', is_found, float(ball.x), float(ball.y), float(ball.z), float(ball.vx), float(ball.vy), float(ball.vz))
            send_packet(PACKET_TYPE_BALL_DATA, ball_payload)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    picam2.stop()
    if 'ser' in locals() and ser.is_open:
        # tell the Teensy the ball is gone, or it keeps acting on the last position
        # (the I term then winds the plate up to its tilt limit)
        for _ in range(3):
            send_packet(PACKET_TYPE_BALL_DATA, struct.pack('<Bffffff', 0, 0, 0, 0, 0, 0, 0))
        ser.flush()
        ser.close()
