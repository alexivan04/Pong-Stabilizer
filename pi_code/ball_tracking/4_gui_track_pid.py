#!/usr/bin/env python3
import cv2
from picamera2 import Picamera2
import numpy as np
import time
import serial
import struct
import math

# --- UART CONFIG ---
START_BYTE = 0xAA
END_BYTE   = 0xBB
PACKET_TYPE_BALL_DATA      = 1
PACKET_TYPE_PID_UPDATE     = 2
PACKET_TYPE_START          = 3
PACKET_TYPE_PATTERN_CHANGE = 4

SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE   = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
except serial.SerialException:
    print("Could not open serial port.")
    exit()

# --- PID SLIDERS ---
def nothing(x): pass

cv2.namedWindow("Ball Tracking")
# Sliders use integers, divided by 1000 before sending (e.g. 185 -> 0.185)
# Default values match Teensy hardcoded: Kp=0.149, Ki=0.112, Kd=0.068
cv2.createTrackbar("P (*1000)", "Ball Tracking", 149, 1000, nothing)
cv2.createTrackbar("I (*1000)", "Ball Tracking", 112, 1000, nothing)
cv2.createTrackbar("D (*1000)", "Ball Tracking", 68,  5000, nothing)
last_p, last_i, last_d = -1, -1, -1

# --- PATTERN STATE ---
# Teensy always sends the real sub-pattern (0-3), never 4.
PATTERN_NAMES = {
    0: "CENTER",
    1: "CIRCLE",
    2: "STAR",
    3: "FIGURE 8",
}
current_pattern      = 0
current_pattern_text = "CENTER"
pattern_start_time   = time.time()

# --- SERIAL SEND ---
def send_packet(packet_type, payload):
    payload_size = len(payload)
    checksum = packet_type ^ payload_size
    for byte in payload:
        checksum ^= byte
    header = struct.pack('<BBB', START_BYTE, packet_type, payload_size)
    footer = struct.pack('<BB', checksum, END_BYTE)
    ser.write(header + payload + footer)

def send_pattern_to_teensy(pattern_idx):
    """Send a pattern change command from Pi to Teensy."""
    send_packet(PACKET_TYPE_PATTERN_CHANGE, struct.pack('<B', pattern_idx))

# --- SERIAL RECEIVE (robust state-machine parser) ---
_rx_buf   = bytearray()
_rx_state = 0  # 0=waiting for START_BYTE, 1=reading packet

def receive_packets():
    global current_pattern, current_pattern_text, pattern_start_time
    global _rx_buf, _rx_state

    while ser.in_waiting > 0:
        b = ser.read(1)[0]

        if _rx_state == 0:
            if b == START_BYTE:
                _rx_buf   = bytearray()
                _rx_state = 1

        elif _rx_state == 1:
            _rx_buf.append(b)

            if len(_rx_buf) < 2:
                continue

            p_type = _rx_buf[0]
            p_size = _rx_buf[1]
            total  = 1 + 1 + p_size + 1 + 1  # type+size+payload+csum+end

            if len(_rx_buf) == total:
                end_byte = _rx_buf[-1]
                rx_csum  = _rx_buf[-2]
                payload  = _rx_buf[2 : 2 + p_size]

                calc_csum = p_type ^ p_size
                for byte in payload:
                    calc_csum ^= byte

                if end_byte == END_BYTE and calc_csum == rx_csum:
                    if p_type == PACKET_TYPE_PATTERN_CHANGE and p_size >= 1:
                        new_pattern = payload[0]
                        if new_pattern != current_pattern:
                            current_pattern      = new_pattern
                            current_pattern_text = PATTERN_NAMES.get(new_pattern, "UNKNOWN")
                            pattern_start_time   = time.time()

                _rx_state = 0
                _rx_buf   = bytearray()

# --- CAMERA CONFIG ---
def load_hsv(filename="hsv_values.txt"):
    try:
        with open(filename, 'r') as f:
            lines = [line.strip() for line in f if line.strip()]
            return (np.array([int(x) for x in lines[0].split(',')]),
                    np.array([int(x) for x in lines[1].split(',')]))
    except Exception:
        return np.array([106, 131, 57]), np.array([123, 255, 165])

LOWER_HSV, UPPER_HSV  = load_hsv()
REAL_BALL_DIAMETER_MM = 40.0
WIDTH, HEIGHT         = 640, 480

try:
    with np.load('cam_calib_fisheye.npz') as data:
        K_fisheye, D_fisheye = data['K'], data['D']
except Exception:
    print("No calibration data found.")
    exit()

new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    K_fisheye, D_fisheye, (WIDTH, HEIGHT), np.eye(3), balance=1.0)
fx, fy = new_K[0, 0], new_K[1, 1]
cx, cy = new_K[0, 2], new_K[1, 2]
map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K_fisheye, D_fisheye, np.eye(3), new_K, (WIDTH, HEIGHT), cv2.CV_16SC2)

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "BGR888"})
picam2.configure(config)
picam2.set_controls({
    "FrameDurationLimits": (int(1e6/60), int(1e6/60)),
    "ExposureTime": 20000,
    "AnalogueGain": 2.0,
    "AeEnable":  False,
    "AwbEnable": False
})
picam2.start()
kernel = np.ones((3, 3), np.uint8)

send_packet(PACKET_TYPE_START, b'')
isStarted = False

# --- TRACKING VARIABLES ---
prev_time = time.time()
prev_smooth_x, prev_smooth_y, prev_smooth_z = 0.0, 0.0, 0.0
vx, vy, vz = 0.0, 0.0, 0.0
POS_ALPHA      = 0.6
VELOCITY_ALPHA = 0.15
DEADBAND       = 1.5
first_detection = True

# ─────────────────────────────────────────────────────────────
# PATTERN VISUALIZER
# ─────────────────────────────────────────────────────────────
PANEL_SIZE   = 170
PANEL_MARGIN = 10
PANEL_STEPS  = 200
MAX_EXTENT   = 70.0

def mm_to_panel_px(x_mm, y_mm):
    half = PANEL_SIZE / 2.0
    px   = int(half + (x_mm / MAX_EXTENT) * (half - 8))
    py   = int(half + (y_mm / MAX_EXTENT) * (half - 8))  # no flip: match Teensy direction
    return (px, py)

def get_pattern_outline(pat):
    pts = []
    if pat == 0:
        pts = [(0.0, 0.0)]
    elif pat == 1:
        omega = (2.0 * math.pi) / 5.0
        for i in range(PANEL_STEPS + 1):
            t = (i / PANEL_STEPS) * 5.0
            pts.append((50.0 * math.cos(omega * t),
                        50.0 * math.sin(omega * t)))
    elif pat == 2:
        for i in range(11):
            angle = (math.pi / 2.0) + i * (math.pi / 5.0)
            r     = 60.0 if (i % 2 == 0) else 25.0
            pts.append((r * math.cos(angle), r * math.sin(angle)))
    elif pat == 3:
        omega = (2.0 * math.pi) / 6.0
        for i in range(PANEL_STEPS + 1):
            t = (i / PANEL_STEPS) * 6.0
            pts.append((60.0 * math.sin(omega * t),
                        30.0 * math.sin(2.0 * omega * t)))
    return [mm_to_panel_px(p[0], p[1]) for p in pts]

def get_pattern_dot(pat, t):
    if pat == 0:
        return mm_to_panel_px(0.0, 0.0)
    elif pat == 1:
        omega = (2.0 * math.pi) / 5.0
        return mm_to_panel_px(50.0 * math.cos(omega * t),
                               50.0 * math.sin(omega * t))
    elif pat == 2:
        period   = 10.0
        mod_t    = math.fmod(t, period)
        seg      = period / 10
        idx      = int(mod_t / seg)
        progress = (mod_t - idx * seg) / seg
        next_idx = (idx + 1) % 10
        def gp(i):
            a = (math.pi / 2.0) + i * (math.pi / 5.0)
            r = 60.0 if (i % 2 == 0) else 25.0
            return r * math.cos(a), r * math.sin(a)
        x1, y1 = gp(idx)
        x2, y2 = gp(next_idx)
        return mm_to_panel_px(x1 + (x2 - x1) * progress,
                               y1 + (y2 - y1) * progress)
    elif pat == 3:
        omega = (2.0 * math.pi) / 6.0
        return mm_to_panel_px(60.0 * math.sin(omega * t),
                               30.0 * math.sin(2.0 * omega * t))
    return mm_to_panel_px(0.0, 0.0)

_cached_outline_pat = -1
_cached_outline     = None

def get_cached_outline(pat):
    global _cached_outline, _cached_outline_pat
    if pat != _cached_outline_pat:
        _cached_outline     = get_pattern_outline(pat)
        _cached_outline_pat = pat
    return _cached_outline

def draw_pattern_panel(frame, t_elapsed, ball_x_mm=None, ball_y_mm=None, ball_found=False):
    pw = PANEL_SIZE
    ph = PANEL_SIZE + 28
    px0 = WIDTH  - pw - PANEL_MARGIN
    py0 = HEIGHT - ph - PANEL_MARGIN

    overlay = frame.copy()
    cv2.rectangle(overlay, (px0, py0), (px0 + pw, py0 + ph), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)
    cv2.rectangle(frame, (px0, py0), (px0 + pw, py0 + ph), (80, 80, 80), 1)

    title = current_pattern_text
    (tw, _), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    cv2.putText(frame, title,
                (px0 + (pw - tw) // 2, py0 + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 255), 1, cv2.LINE_AA)

    draw_top = py0 + 26
    mid_x    = px0 + pw // 2
    mid_y    = draw_top + PANEL_SIZE // 2
    cv2.line(frame, (px0 + 4, mid_y),         (px0 + pw - 4, mid_y),              (50, 50, 50), 1)
    cv2.line(frame, (mid_x, draw_top + 4),    (mid_x, draw_top + PANEL_SIZE - 4), (50, 50, 50), 1)

    def to_frame(p):
        return (p[0] + px0, p[1] + draw_top)

    # --- shape outline ---
    outline = get_cached_outline(current_pattern)
    trail   = [to_frame(p) for p in outline]

    if len(trail) > 1:
        pts_np = np.array(trail, dtype=np.int32).reshape((-1, 1, 2))
        closed = current_pattern in (1, 2)
        cv2.polylines(frame, [pts_np], isClosed=closed,
                      color=(0, 160, 255), thickness=1, lineType=cv2.LINE_AA)

    # --- animated target dot (green) ---
    dot = to_frame(get_pattern_dot(current_pattern, t_elapsed))
    if current_pattern == 0:
        cv2.drawMarker(frame, dot, (0, 255, 180), cv2.MARKER_CROSS, 12, 1, cv2.LINE_AA)
    else:
        cv2.circle(frame, dot, 5, (0, 255, 180),  -1, cv2.LINE_AA)
        cv2.circle(frame, dot, 5, (255, 255, 255),  1, cv2.LINE_AA)

    # --- real ball position (yellow dot) ---
    if ball_found and ball_x_mm is not None and ball_y_mm is not None:
        ball_px = to_frame(mm_to_panel_px(ball_x_mm, ball_y_mm))
        # clamp inside panel bounds
        ball_px = (
            max(px0 + 4, min(px0 + pw - 4, ball_px[0])),
            max(draw_top + 4, min(draw_top + PANEL_SIZE - 4, ball_px[1]))
        )
        cv2.circle(frame, ball_px, 5, (0, 220, 255),  -1, cv2.LINE_AA)
        cv2.circle(frame, ball_px, 5, (255, 255, 255),  1, cv2.LINE_AA)

# ─────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────
try:
    while True:
        receive_packets()

        # --- PID SLIDER HANDLING ---
        p_val = cv2.getTrackbarPos("P (*1000)", "Ball Tracking")
        i_val = cv2.getTrackbarPos("I (*1000)", "Ball Tracking")
        d_val = cv2.getTrackbarPos("D (*1000)", "Ball Tracking")
        if p_val != last_p or i_val != last_i or d_val != last_d:
            pid_payload = struct.pack('<fff',
                p_val / 1000.0,
                i_val / 1000.0,
                d_val / 1000.0)
            send_packet(PACKET_TYPE_PID_UPDATE, pid_payload)
            last_p, last_i, last_d = p_val, i_val, d_val

        raw_frame = picam2.capture_array()
        frame     = cv2.remap(raw_frame, map1, map2, cv2.INTER_LINEAR)

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
        mask = cv2.erode(mask,  kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        is_found = 0
        X_real, Y_real, Z_depth = 0.0, 0.0, 0.0
        draw_x, draw_y, radius  = 0, 0, 0

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius_f) = cv2.minEnclosingCircle(c)
            if radius_f > 5:
                is_found = 1
                draw_x, draw_y, radius = int(x), int(y), int(radius_f)
                Z_pinhole        = (fx * REAL_BALL_DIAMETER_MM) / (radius_f * 2)
                dist_from_center = math.sqrt((x - cx)**2 + (y - cy)**2)
                Z_depth          = Z_pinhole * math.sqrt(1 + (dist_from_center / fx)**2)
                X_real           = ((x - cx) * Z_depth) / fx
                Y_real           = ((y - cy) * Z_depth) / fy

        curr_time = time.time()
        dt = curr_time - prev_time
        if dt <= 0: dt = 0.016

        if is_found:
            if first_detection:
                prev_smooth_x, prev_smooth_y, prev_smooth_z = X_real, Y_real, Z_depth
                first_detection = False
            smooth_x = POS_ALPHA * X_real  + (1.0 - POS_ALPHA) * prev_smooth_x
            smooth_y = POS_ALPHA * Y_real  + (1.0 - POS_ALPHA) * prev_smooth_y
            smooth_z = POS_ALPHA * Z_depth + (1.0 - POS_ALPHA) * prev_smooth_z
            dx = smooth_x - prev_smooth_x
            dy = smooth_y - prev_smooth_y
            dz = smooth_z - prev_smooth_z
            if abs(dx) < DEADBAND: dx = 0.0
            if abs(dy) < DEADBAND: dy = 0.0
            if abs(dz) < DEADBAND: dz = 0.0
            vx = VELOCITY_ALPHA * (dx / dt) + (1.0 - VELOCITY_ALPHA) * vx
            vy = VELOCITY_ALPHA * (dy / dt) + (1.0 - VELOCITY_ALPHA) * vy
            vz = VELOCITY_ALPHA * (dz / dt) + (1.0 - VELOCITY_ALPHA) * vz
            prev_smooth_x, prev_smooth_y, prev_smooth_z = smooth_x, smooth_y, smooth_z
        else:
            vx *= 0.8; vy *= 0.8; vz *= 0.8
            first_detection = True

        prev_time = curr_time

        if is_found and not isStarted:
            send_packet(PACKET_TYPE_START, b'')
            isStarted = True
        if isStarted:
            ball_payload = struct.pack('<Bffffff',
                is_found,
                float(X_real), float(Y_real), float(Z_depth),
                float(vx),     float(vy),     float(vz))
            send_packet(PACKET_TYPE_BALL_DATA, ball_payload)

        # GUI
        if is_found:
            cv2.circle(frame, (draw_x, draw_y), radius, (0, 255, 0), 2)
            cv2.circle(frame, (draw_x, draw_y), 5, (0, 0, 255), -1)

        status = "FOUND" if is_found else "LOST"
        col    = (0, 255, 0) if is_found else (0, 0, 255)
        cv2.putText(frame, f"Status: {status}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        cv2.putText(frame, f"X: {X_real:.1f} mm",  (20,  60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Y: {Y_real:.1f} mm",  (20,  90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Z: {Z_depth:.1f} mm", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Vx: {vx:.1f}", (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        cv2.putText(frame, f"Vy: {vy:.1f}", (20, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        cv2.putText(frame, f"Vz: {vz:.1f}", (20, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        # current PID values (read from sliders)
        cur_p = cv2.getTrackbarPos("P (*1000)", "Ball Tracking") / 1000.0
        cur_i = cv2.getTrackbarPos("I (*1000)", "Ball Tracking") / 1000.0
        cur_d = cv2.getTrackbarPos("D (*1000)", "Ball Tracking") / 1000.0
        cv2.putText(frame, f"P: {cur_p:.3f}", (20, 265),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 255, 100), 2)
        cv2.putText(frame, f"I: {cur_i:.3f}", (20, 295),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 255, 100), 2)
        cv2.putText(frame, f"D: {cur_d:.3f}", (20, 325),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 255, 100), 2)

        # key hints overlay (bottom-left, above pattern panel)
        hints = [
            "1:CENTER  2:CIRCLE",
            "3:STAR    4:FIG.8",
            "5:CYCLE   ESC:quit",
        ]
        for hi, htxt in enumerate(hints):
            cv2.putText(frame, htxt,
                        (20, HEIGHT - 15 - (len(hints) - 1 - hi) * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1, cv2.LINE_AA)

        t_elapsed = time.time() - pattern_start_time
        draw_pattern_panel(frame, t_elapsed,
                           ball_x_mm=X_real, ball_y_mm=Y_real,
                           ball_found=bool(is_found))

        cv2.imshow("Ball Tracking", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:      # ESC
            break
        elif key == ord('1'):
            send_pattern_to_teensy(0)
            current_pattern      = 0
            current_pattern_text = PATTERN_NAMES[0]
            pattern_start_time   = time.time()
        elif key == ord('2'):
            send_pattern_to_teensy(1)
            current_pattern      = 1
            current_pattern_text = PATTERN_NAMES[1]
            pattern_start_time   = time.time()
        elif key == ord('3'):
            send_pattern_to_teensy(2)
            current_pattern      = 2
            current_pattern_text = PATTERN_NAMES[2]
            pattern_start_time   = time.time()
        elif key == ord('4'):
            send_pattern_to_teensy(3)
            current_pattern      = 3
            current_pattern_text = PATTERN_NAMES[3]
            pattern_start_time   = time.time()
        elif key == ord('5'):
            # CYCLE ALL — send index 4 to Teensy, Pi waits for
            # Teensy to echo back the active sub-pattern
            send_pattern_to_teensy(4)
            current_pattern_text = "CYCLE ALL"

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    picam2.stop()
    if 'ser' in locals() and ser.is_open:
        ser.close()
    cv2.destroyAllWindows()
