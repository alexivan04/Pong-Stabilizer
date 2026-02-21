import cv2
from picamera2 import Picamera2
import numpy as np
import time
import serial
import struct

# --- COMMUNICATION PROTOCOL ---
START_BYTE = 0xAA
END_BYTE = 0xBB
PACKET_TYPE_BALL_DATA = 1

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

# --- UART SETUP ---
SERIAL_PORT = '/dev/ttyAMA0' # Or /dev/serial0
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} bps.")
except serial.SerialException as e:
    print(f"FATAL ERROR: Could not open serial port {SERIAL_PORT}: {e}")
    print("Check wiring, permissions (sudo adduser $USER dialout), and raspi-config settings.")
    exit()

# Load Calibration
try:
    with np.load('cam_calib.npz') as data:
        mtx, dist = data['mtx'], data['dist']
        fx, fy = mtx[0, 0], mtx[1, 1]
        cx, cy = mtx[0, 2], mtx[1, 2]
except FileNotFoundError:
    print("FATAL ERROR: Calibration file 'cam_calib.npz' not found.")
    exit()

def send_packet(packet_type, payload):
    """Constructs and sends a complete packet over UART."""
    payload_size = len(payload)
    
    # Calculate checksum (simple XOR sum)
    checksum = packet_type ^ payload_size
    for byte in payload:
        checksum ^= byte
        
    # Pack header, payload, and footer into a binary packet
    # < = Little-endian, B = unsigned char
    header = struct.pack('<BBB', START_BYTE, packet_type, payload_size)
    footer = struct.pack('<BB', checksum, END_BYTE)
    
    packet = header + payload + footer
    ser.write(packet)

# --- picamera2 setup ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (WIDTH, HEIGHT), "format": "BGR888"})
picam2.configure(config)
picam2.set_controls({
    "FrameDurationLimits": (int(1e6/FRAMERATE), int(1e6/FRAMERATE)),
    "ExposureTime": SHUTTER, "AnalogueGain": GAIN,
    "AeEnable": False, "AwbEnable": False,
})

kernel = np.ones((3,3), np.uint8)
print("--- UART Ball Tracker Initialized ---")
picam2.start()
time.sleep(1)
print("Tracking started. Press Ctrl+C to stop.")

try:
    while True:
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball_found = False
        X_real, Y_real, Z_depth = 0.0, 0.0, 0.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 5:
                ball_found = True
                point_distorted = np.array([[[x, y]]], dtype=np.float64)
                point_undistorted = cv2.undistortPoints(point_distorted, mtx, dist, P=mtx)
                ux, uy = point_undistorted[0][0]
                
                pixel_diameter = radius * 2
                Z_depth = (fx * REAL_BALL_DIAMETER_MM) / pixel_diameter
                X_real = ((ux - cx) * Z_depth) / fx
                Y_real = ((uy - cy) * Z_depth) / fy

        # --- PACK AND SEND DATA ---
        # '<' for little-endian, 'B' for uint8_t (our boolean), 'f' for float
        payload = struct.pack('<Bfff', 1 if ball_found else 0, X_real, Y_real, Z_depth)
        send_packet(PACKET_TYPE_BALL_DATA, payload)
        
        # Optional: Print what's being sent for debugging
        # print(f"Sent: Found={ball_found}, X={X_real:.1f}, Y={Y_real:.1f}, Z={Z_depth:.1f}")
        
        # Small delay to not overwhelm the serial buffer, can be adjusted
        time.sleep(0.005)

except KeyboardInterrupt:
    print("\nProgram stopped by user.")
finally:
    print("Shutting down.")
    picam2.stop()
    ser.close()
