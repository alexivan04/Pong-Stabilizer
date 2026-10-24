#!/usr/bin/env python3
import cv2
import numpy as np
import time
import serial
import struct
import signal
import math
import os

from ball_detector import WIDTH, HEIGHT, PATTERN_SCALE, UNIT_SCALE, create_tracker, load_undistort, start_camera

def _on_sigterm(signum, frame):
    raise KeyboardInterrupt
signal.signal(signal.SIGTERM, _on_sigterm)

# --- UART CONFIG ---
START_BYTE = 0xAA
END_BYTE   = 0xBB
PACKET_TYPE_BALL_DATA      = 1
PACKET_TYPE_PID_UPDATE     = 2
PACKET_TYPE_START          = 3
PACKET_TYPE_PATTERN_CHANGE = 4
PACKET_TYPE_TARGET_POS     = 5
# Teensy's plate-height range in deg above the low stop (HEIGHT_MIN/MAX_DEG in inverseKinematics.h)
PLATE_HEIGHT_MIN_DEG, PLATE_HEIGHT_MAX_DEG = 15.0, 30.0

SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE   = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
except serial.SerialException:
    print("Could not open serial port.")
    exit()

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
target_x, target_y   = 0.0, 0.0   # Teensy's current target, from TARGET_POS packets
plate_height_deg     = None       # Teensy's plate height setting (newer firmware only)

# --- OPTIONAL RUN LOG (TRACK_LOG=/path/dir): per-frame CSV + a frame every 2 s ---
LOG_DIR = os.environ.get("TRACK_LOG")
log_file, last_log_frame = None, 0.0
if LOG_DIR:
    os.makedirs(LOG_DIR, exist_ok=True)
    log_file = open(os.path.join(LOG_DIR, "track.csv"), "w", buffering=1)
    log_file.write("t,found,x,y,z,vx,vy,target_x,target_y,pattern,height_deg\n")

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
    global target_x, target_y, plate_height_deg
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
                    elif p_type == PACKET_TYPE_TARGET_POS and p_size == 8:
                        target_x, target_y = struct.unpack('<ff', bytes(payload))
                    elif p_type == PACKET_TYPE_TARGET_POS and p_size == 12:
                        target_x, target_y, plate_height_deg = struct.unpack('<fff', bytes(payload))

                _rx_state = 0
                _rx_buf   = bytearray()

# --- CAMERA CONFIG ---
map1, map2, new_K = load_undistort()
picam2  = start_camera()
tracker = create_tracker(new_K)

send_packet(PACKET_TYPE_START, b'')
isStarted   = False
show_mask   = False

# ─────────────────────────────────────────────────────────────
# PATTERN VISUALIZER
# ─────────────────────────────────────────────────────────────
PANEL_SIZE   = 170
PANEL_MARGIN = 10
PANEL_STEPS  = 200
MAX_EXTENT   = 70.0 * UNIT_SCALE
# Teensy pattern sizes (main.cpp), same units as the ball position
CIRCLE_R     = 50.0 * PATTERN_SCALE
STAR_R_OUT   = 60.0 * PATTERN_SCALE
STAR_R_IN    = 25.0 * PATTERN_SCALE
FIG8_RX      = 60.0 * PATTERN_SCALE
FIG8_RY      = 30.0 * PATTERN_SCALE

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
            pts.append((CIRCLE_R * math.cos(omega * t),
                        CIRCLE_R * math.sin(omega * t)))
    elif pat == 2:
        for i in range(11):
            angle = (math.pi / 2.0) + i * (math.pi / 5.0)
            r     = STAR_R_OUT if (i % 2 == 0) else STAR_R_IN
            pts.append((r * math.cos(angle), r * math.sin(angle)))
    elif pat == 3:
        omega = (2.0 * math.pi) / 6.0
        for i in range(PANEL_STEPS + 1):
            t = (i / PANEL_STEPS) * 6.0
            pts.append((FIG8_RX * math.sin(omega * t),
                        FIG8_RY * math.sin(2.0 * omega * t)))
    return [mm_to_panel_px(p[0], p[1]) for p in pts]

def get_pattern_dot(pat, t):
    if pat == 0:
        return mm_to_panel_px(0.0, 0.0)
    elif pat == 1:
        omega = (2.0 * math.pi) / 5.0
        return mm_to_panel_px(CIRCLE_R * math.cos(omega * t),
                               CIRCLE_R * math.sin(omega * t))
    elif pat == 2:
        period   = 10.0
        mod_t    = math.fmod(t, period)
        seg      = period / 10
        idx      = int(mod_t / seg)
        progress = (mod_t - idx * seg) / seg
        next_idx = (idx + 1) % 10
        def gp(i):
            a = (math.pi / 2.0) + i * (math.pi / 5.0)
            r = STAR_R_OUT if (i % 2 == 0) else STAR_R_IN
            return r * math.cos(a), r * math.sin(a)
        x1, y1 = gp(idx)
        x2, y2 = gp(next_idx)
        return mm_to_panel_px(x1 + (x2 - x1) * progress,
                               y1 + (y2 - y1) * progress)
    elif pat == 3:
        omega = (2.0 * math.pi) / 6.0
        return mm_to_panel_px(FIG8_RX * math.sin(omega * t),
                               FIG8_RY * math.sin(2.0 * omega * t))
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
    # the Teensy's actual target (TARGET_POS packets): recomputing it here from the
    # Pi's clock put it out of phase, e.g. on the opposite side of the figure-8
    dot = to_frame(mm_to_panel_px(target_x, target_y))
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

        raw_frame = picam2.capture_array()
        frame     = cv2.remap(raw_frame, map1, map2, cv2.INTER_LINEAR)

        ball = tracker.update(frame)
        is_found = 1 if ball.found else 0
        X_real, Y_real, Z_depth = ball.x, ball.y, ball.z
        vx, vy, vz = ball.vx, ball.vy, ball.vz
        draw_x, draw_y, radius = int(ball.u), int(ball.v), int(ball.r)

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
        clean_frame = frame.copy()   # undrawn, for the 'd' snapshot
        cv2.circle(frame, (int(tracker.roi[0]), int(tracker.roi[1])), int(tracker.roi[2]),
                   (0, 200, 255), 1)
        if is_found:
            ring_col = (0, 200, 255) if ball.coasting else (0, 255, 0)
            cv2.circle(frame, (draw_x, draw_y), radius, ring_col, 2)
            cv2.circle(frame, (draw_x, draw_y), 5, (0, 0, 255), -1)

        status = ("PREDICTED" if ball.coasting else "FOUND") if is_found else "LOST"
        col    = (0, 255, 0) if is_found else (0, 0, 255)
        cv2.putText(frame, f"Status: {status}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        cv2.putText(frame, f"X: {X_real:.1f} mm",  (20,  60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Y: {Y_real:.1f} mm",  (20,  90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Z: {Z_depth:.1f} mm", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if plate_height_deg is not None:
            at_limit = (plate_height_deg <= PLATE_HEIGHT_MIN_DEG + 0.05 or
                        plate_height_deg >= PLATE_HEIGHT_MAX_DEG - 0.05)
            cv2.putText(frame, f"Plate: {plate_height_deg:.1f} deg" + (" (limit)" if at_limit else ""),
                        (20, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255) if at_limit else (255, 255, 255), 2)
        cv2.putText(frame, f"Vx: {vx:.1f}", (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        cv2.putText(frame, f"Vy: {vy:.1f}", (20, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        cv2.putText(frame, f"Vz: {vz:.1f}", (20, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        # key hints overlay
        hints = [
            "1:CENTER  2:CIRCLE",
            "3:STAR    4:FIG.8",
            "5:CYCLE   M:mask  D:snapshot  ESC:quit",
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
        if log_file is not None:
            now = time.time()
            log_file.write(f"{now:.3f},{is_found},{X_real:.1f},{Y_real:.1f},{Z_depth:.1f},"
                           f"{vx:.0f},{vy:.0f},{target_x:.1f},{target_y:.1f},{current_pattern},"
                           f"{plate_height_deg if plate_height_deg is not None else float('nan'):.2f}\n")
            if now - last_log_frame >= 2.0:
                last_log_frame = now
                cv2.imwrite(os.path.join(LOG_DIR, "latest.jpg"), frame)
        if show_mask and tracker.mask is not None:
            cv2.imshow("Mask", tracker.mask)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord('m'):
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow("Mask")
        elif key == ord('d'):
            stamp = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"snapshot_{stamp}_frame.png", clean_frame)
            if tracker.mask is not None:
                cv2.imwrite(f"snapshot_{stamp}_mask.png", tracker.mask)
            print(f"Saved snapshot_{stamp}_*.png")
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
            send_pattern_to_teensy(4)
            current_pattern_text = "CYCLE ALL"

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
    if log_file is not None:
        log_file.close()
    cv2.destroyAllWindows()
