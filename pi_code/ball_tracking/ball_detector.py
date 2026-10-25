"""
Ball detection + tracking, shared by the tracking scripts.

DETECTOR_MODE picks the method:

  "auto"     No colour calibration. The ball is found as the round, ball-sized
             region of the plate whose outline stays the same while the
             saturation threshold is lowered (a maximally stable region, as in
             MSER). The threshold is chosen per frame, so brightness, colour
             temperature and backlight haze don't need re-tuning.

  "sliders"  HSV window from hsv_values.txt (tune with 1_tune_color.py), only
             searching the plate. Keep the V range wide and tune H/S.

Both use the same position/velocity smoothing as the original scripts.

Typical use:
    map1, map2, new_K = load_undistort()
    picam2  = start_camera()
    tracker = create_tracker(new_K)
    ...
    frame = cv2.remap(picam2.capture_array(), map1, map2, cv2.INTER_LINEAR)
    ball  = tracker.update(frame)      # BallState, positions in mm
"""
import math
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

DETECTOR_MODE = "auto"   # "auto" or "sliders", see above

# Only accept a ball whose centre is on the plate. Outside it there's wood, skin
# and a side panel with the same hue as the ball. Defined in mm, so it fits at
# any plate height: the height depends on where the arms rest at power-on (the
# plate was ~125 mm from the camera in one session, ~195 mm in another), and it's
# converted to pixels with the camera-to-plate distance measured from the ball.
# Both sessions give the same plate: flat edges ~110 mm from its centre.
PLATE_CENTER_PX  = (324, 236)  # where the camera sees the plate centre
# Ball positions are reported relative to the plate centre, so the CENTER target
# and the patterns sit in the middle of the plate. The camera's optical centre is
# this far from it (measured from markers on the brackets with the plate level at
# working height, 2026-10-25; see experiments/plate_markers on that branch).
PLATE_CENTRE_MM  = (8.0, -1.2)
PLATE_RADIUS_MM  = 117.0       # ball centre must be within this of it
ROI_MARGIN_MM    = 22.0        # pixels are searched this far beyond, so a ball at the edge stays whole
PLATE_Z_START_MM = 125.0       # camera-to-plate distance assumed until the ball is seen (low = generous)
PLATE_Z_ALPHA    = 0.02        # how fast that estimate follows the measured ball distance
Z_MIN_MM, Z_MAX_MM = 90.0, 320.0  # plausible camera-to-ball distances, sets the plausible ball size
# Z comes from the ball's radius, and at ~200 mm the ball is only ~13 px, so a
# 1 px radius error is ~7% of Z (measured: Z jumped up to 70 mm between frames).
# X/Y are scaled by Z, so they jumped with it and the D term kicked the plate.
Z_MEDIAN_XY  = 15   # frames of Z whose median scales X/Y (0.25 s; the plate height barely moves)
Z_MEDIAN_OUT = 5    # frames for the reported Z: drops single-frame spikes, ~2 frames delay

LEGACY_HSV_FILE = "hsv_values.txt"
CALIB_FILE      = "cam_calib_fisheye.npz"

# Defaults for the slider window, measured on the orange ball in real frames.
# Saturation is what separates it: ball S 140-225, skin/plate haze/side panel
# S 60-110 at the *same* hue. V is left wide open so room brightness doesn't
# matter (the old V window was what needed re-tuning whenever the light changed).
DEFAULT_HSV_LOWER = (98, 145, 40)
DEFAULT_HSV_UPPER = (116, 255, 255)

# --- AUTO MODE ---
MIN_FILL      = 0.6     # convex hull area / enclosing-circle area: only round blobs count
                        # (the hull bridges the gap a glare spot cuts into the ball's edge)
AUTO_T_STEP   = 6       # saturation threshold step of the stability search
AUTO_T_MIN    = 50      # never go below this: plate haze and skin live there
MIN_PEAK_SAT  = 100     # the ball's inner 90th-percentile saturation is >= 113 in every test
                        # condition (87 once under a torch); a dull brown patch seen through
                        # the plate's corner reaches only 72-74 but is round and stable
AUTO_T_BAND   = 30      # while tracking, only search +- this around the last threshold
DARK_V        = 25      # pixels darker than this have meaningless saturation

# --- CAMERA (shared so tuning and tracking see identical images) ---
WIDTH, HEIGHT = 640, 480
FRAMERATE     = 60
EXPOSURE_US   = 20000   # same as 1_tune_color.py (the camera caps it at the 16.6 ms frame time)
ANALOGUE_GAIN = 2.0

REAL_BALL_DIAMETER_MM = 40.0

# Positions are real mm with the auto detector (Z ~145 mm, as measured by hand);
# the old slider settings over-estimated the ball radius and reported ~1/1.4 of
# that. Gains and distances tuned in the old units are scaled by this. Keep in
# sync with UNIT_SCALE in teensy_code/include/inverseKinematics.h.
UNIT_SCALE = 1.4

# Size of the Teensy's target patterns (main.cpp), only used for the GUI minimap
# here. Keep in sync with PATTERN_SCALE in inverseKinematics.h.
PATTERN_SCALE = 1.0


# =============================================================
# Camera helpers
# =============================================================
def start_camera():
    from picamera2 import Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "BGR888"})
    picam2.configure(config)
    picam2.set_controls({
        "FrameDurationLimits": (int(1e6 / FRAMERATE), int(1e6 / FRAMERATE)),
        "ExposureTime": EXPOSURE_US,
        "AnalogueGain": ANALOGUE_GAIN,
        "AeEnable":  False,
        "AwbEnable": False,
    })
    picam2.start()
    return picam2


def load_undistort(filename=CALIB_FILE):
    """Returns (map1, map2, new_K) for cv2.remap, exits if calibration is missing."""
    try:
        with np.load(filename) as data:
            K_fisheye, D_fisheye = data['K'], data['D']
    except Exception:
        raise SystemExit(f"FATAL ERROR: '{filename}' not found.")
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K_fisheye, D_fisheye, (WIDTH, HEIGHT), np.eye(3), balance=1.0)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K_fisheye, D_fisheye, np.eye(3), new_K, (WIDTH, HEIGHT), cv2.CV_16SC2)
    return map1, map2, new_K


def load_hsv_range(filename=LEGACY_HSV_FILE):
    """(lower, upper) HSV arrays from the slider tuner."""
    try:
        with open(filename, 'r') as f:
            lines = [line.strip() for line in f if line.strip()]
            return (np.array([int(x) for x in lines[0].split(',')]),
                    np.array([int(x) for x in lines[1].split(',')]))
    except Exception:
        return np.array(DEFAULT_HSV_LOWER), np.array(DEFAULT_HSV_UPPER)


def plate_roi(fx, plate_z_mm):
    """(u, v, r) in pixels: the circle the ball centre must be in, at that plate distance."""
    return PLATE_CENTER_PX[0], PLATE_CENTER_PX[1], PLATE_RADIUS_MM * fx / plate_z_mm


def plate_roi_mask(fx, plate_z_mm):
    """255 where pixels are searched: the plate circle plus ROI_MARGIN_MM."""
    u, v, r = plate_roi(fx, plate_z_mm)
    r += ROI_MARGIN_MM * fx / plate_z_mm
    mask = np.zeros((HEIGHT, WIDTH), np.uint8)
    cv2.circle(mask, (int(u), int(v)), int(r), 255, -1)
    return mask


def create_tracker(new_K):
    """The tracker selected by DETECTOR_MODE; both return BallState from update(frame)."""
    if DETECTOR_MODE == "sliders":
        return SliderTracker(new_K)
    return AutoTracker(new_K)


# =============================================================
# Tracking
# =============================================================
@dataclass
class BallState:
    found: bool = False
    coasting: bool = False   # kept for the GUIs; these trackers never predict
    x: float = 0.0           # mm, relative to the optical centre
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0          # mm/s
    vy: float = 0.0
    vz: float = 0.0
    u: float = 0.0           # pixels, for drawing
    v: float = 0.0
    r: float = 0.0


class SliderTracker:
    """The original detection: HSV window from 1_tune_color.py, largest blob,
    EMA position/velocity smoothing with a deadband. Only searches the plate."""

    POS_ALPHA      = 0.6
    VELOCITY_ALPHA = 0.15
    DEADBAND       = 1.5

    def __init__(self, new_K, hsv_file=LEGACY_HSV_FILE):
        self.lower, self.upper = load_hsv_range(hsv_file)
        self.fx, self.fy = new_K[0, 0], new_K[1, 1]
        self.cx, self.cy = new_K[0, 2], new_K[1, 2]
        self.kernel = np.ones((3, 3), np.uint8)
        ball_r = REAL_BALL_DIAMETER_MM / 2
        self.r_min = 0.8 * self.fx * ball_r / Z_MAX_MM   # plausible ball radius on screen
        self.r_max = 1.2 * self.fx * ball_r / Z_MIN_MM
        self.plate_z = PLATE_Z_START_MM
        self._set_roi()
        self._mask = None
        self.prev_time = time.time()
        self.prev_smooth = None
        self.vx = self.vy = self.vz = 0.0
        self.z_hist = deque(maxlen=Z_MEDIAN_XY)

    @property
    def mask(self):
        """Last binary mask, for debug display."""
        return self._mask

    def _set_roi(self):
        """Plate circle in pixels for the current camera-to-plate distance."""
        self.roi = plate_roi(self.fx, self.plate_z)
        self.roi_mask = plate_roi_mask(self.fx, self.plate_z)

    def _on_plate(self, u, v):
        return math.hypot(u - self.roi[0], v - self.roi[1]) <= self.roi[2]

    def update(self, frame, now=None):
        return self._state(self._detect(frame), now)

    def _detect(self, frame):
        """(u, v, radius) in pixels, or None."""
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.bitwise_and(mask, self.roi_mask)
        mask = cv2.erode(mask,  self.kernel, iterations=1)
        mask = cv2.dilate(mask, self.kernel, iterations=2)
        self._mask = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        circles = [cv2.minEnclosingCircle(c) + (cv2.contourArea(c),) for c in contours]
        circles = [c for c in circles if self._on_plate(*c[0])]
        if not circles:
            return None
        ((x, y), radius, _) = max(circles, key=lambda c: c[2])
        return (x, y, radius) if radius > 5 else None

    def _state(self, det, now):
        found = det is not None
        X_real, Y_real, Z_depth = 0.0, 0.0, 0.0
        x = y = radius = 0.0
        if found:
            x, y, radius = det
            Z_pinhole        = (self.fx * REAL_BALL_DIAMETER_MM) / (radius * 2)
            dist_from_center = math.sqrt((x - self.cx)**2 + (y - self.cy)**2)
            Z_raw            = Z_pinhole * math.sqrt(1 + (dist_from_center / self.fx)**2)
            if Z_MIN_MM <= Z_raw <= Z_MAX_MM:
                self.z_hist.append(Z_raw)
            z_recent = list(self.z_hist) or [Z_raw]
            Z_depth          = float(np.median(z_recent[-Z_MEDIAN_OUT:]))
            Z_scale          = float(np.median(z_recent))
            # relative to the plate centre, not the camera's optical centre
            X_real           = ((x - self.cx) * Z_scale) / self.fx - PLATE_CENTRE_MM[0]
            Y_real           = ((y - self.cy) * Z_scale) / self.fy - PLATE_CENTRE_MM[1]
            if Z_MIN_MM <= Z_depth <= Z_MAX_MM:
                # follow the plate height, so the plate circle keeps fitting the plate
                plate_z = Z_depth - REAL_BALL_DIAMETER_MM / 2
                self.plate_z += PLATE_Z_ALPHA * (plate_z - self.plate_z)
                if abs(plate_roi(self.fx, self.plate_z)[2] - self.roi[2]) >= 2:
                    self._set_roi()

        now = time.time() if now is None else now
        dt = now - self.prev_time
        if dt <= 0: dt = 0.016

        if found:
            if self.prev_smooth is None:
                self.prev_smooth = (X_real, Y_real, Z_depth)
            px, py, pz = self.prev_smooth
            smooth = (self.POS_ALPHA * X_real  + (1.0 - self.POS_ALPHA) * px,
                      self.POS_ALPHA * Y_real  + (1.0 - self.POS_ALPHA) * py,
                      self.POS_ALPHA * Z_depth + (1.0 - self.POS_ALPHA) * pz)
            d = [s - p for s, p in zip(smooth, self.prev_smooth)]
            d = [0.0 if abs(di) < self.DEADBAND else di for di in d]
            a = self.VELOCITY_ALPHA
            self.vx = a * (d[0] / dt) + (1.0 - a) * self.vx
            self.vy = a * (d[1] / dt) + (1.0 - a) * self.vy
            self.vz = a * (d[2] / dt) + (1.0 - a) * self.vz
            self.prev_smooth = smooth
        else:
            self.vx *= 0.8; self.vy *= 0.8; self.vz *= 0.8
            self.prev_smooth = None
        self.prev_time = now

        # like before: raw position is sent, only velocity is smoothed
        return BallState(found=found, x=X_real, y=Y_real, z=Z_depth,
                         vx=self.vx, vy=self.vy, vz=self.vz,
                         u=x, v=y, r=radius)


def saturation(img):
    """HSV-style saturation 0-255 (any channel order); 0 where too dark to tell."""
    f = img.astype(np.float32)
    mx, mn = f.max(axis=2), f.min(axis=2)
    s = np.where(mx > 1, 255.0 * (mx - mn) / np.maximum(mx, 1), 0)
    s[mx < DARK_V] = 0
    return np.clip(s, 0, 255).astype(np.uint8)


class AutoTracker(SliderTracker):
    """Calibration-free detection: the most stable round saturated blob on the plate.

    Lowering a saturation threshold step by step, the ball appears as a disk whose
    area barely changes over many steps, until the threshold reaches the haze/skin
    level and it melts into them. The step where it changes least is picked, so the
    threshold adapts to every frame's lighting by itself.

    Searching the whole plate costs ~17 ms on a Pi 5, so it's only done when the
    ball is lost; while tracking, a window around the ball and a threshold band
    around last frame's value are searched (~2 ms).
    """

    def __init__(self, new_K):
        super().__init__(new_K)
        self.last = None      # (u, v, r) of the last detection
        self.t = None         # its stable saturation threshold
        self._frame = None

    @property
    def mask(self):
        """The ball mask at the current threshold (only computed when displayed)."""
        if self._frame is None or self.t is None:
            return None
        m = (saturation(self._frame) >= self.t).astype(np.uint8) * 255
        return cv2.bitwise_and(m, self.roi_mask)

    def _detect(self, frame):
        self._frame = frame
        if self.last is not None:
            u0, v0, r0 = self.last
            half = int(max(2.5 * r0, 50))
            band = None if self.t is None else (self.t - AUTO_T_BAND, self.t + AUTO_T_BAND)
            res = self._search(frame, int(u0) - half, int(v0) - half,
                               int(u0) + half, int(v0) + half, band)
            if res is not None:
                return self.last
        u, v, r = self.roi
        r += ROI_MARGIN_MM * self.fx / self.plate_z
        res = self._search(frame, int(u - r), int(v - r), int(u + r), int(v + r))
        if res is None:
            self.last = self.t = None
            return None
        return self.last

    def _search(self, frame, x0, y0, x1, y1, t_range=None):
        x0, y0, x1, y1 = max(0, x0), max(0, y0), min(WIDTH, x1), min(HEIGHT, y1)
        roi = self.roi_mask[y0:y1, x0:x1]
        if x1 - x0 < 10 or y1 - y0 < 10 or not roi.any():
            return None
        s = cv2.bitwise_and(saturation(frame[y0:y1, x0:x1]), roi)
        res = self._stable_blob(s, roi, t_range, x0, y0)
        if res is None:
            return None
        u, v, r, t = res
        self.last, self.t = (u + x0, v + y0, r), t
        return self.last

    def _stable_blob(self, s, roi, t_range, x0, y0):
        """(u, v, r, t) in crop coordinates; (x0, y0) is the crop's offset in the frame."""
        top = int(np.percentile(s[roi > 0], 99.9))
        hi, lo = (top, AUTO_T_MIN) if t_range is None else \
                 (min(top, t_range[1]), max(AUTO_T_MIN, t_range[0]))

        # follow every candidate blob down through the thresholds
        tracks = []   # each: [(t, area, u, v, r, fill), ...]
        for t in range(hi, lo, -AUTO_T_STEP):
            m = (s >= t).astype(np.uint8) * 255
            m = cv2.dilate(cv2.erode(m, self.kernel, iterations=1), self.kernel, iterations=2)
            for c in cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                a = cv2.contourArea(cv2.convexHull(c))
                if a < math.pi * self.r_min ** 2 * 0.4:
                    continue
                (u, v), r = cv2.minEnclosingCircle(c)
                if not (self.r_min <= r <= self.r_max) or not self._on_plate(u + x0, v + y0):
                    continue
                entry = (t, a, u, v, r, a / (math.pi * r * r))
                for track in tracks:
                    pt, _, pu, pv, _, _ = track[-1]
                    if pt == t + AUTO_T_STEP and math.hypot(u - pu, v - pv) < max(8, 0.5 * r):
                        track.append(entry)
                        break
                else:
                    tracks.append([entry])

        # most stable = least relative area growth between neighbouring thresholds,
        # counted only while the blob is round; ties go to the more saturated one
        candidates = []
        for track in tracks:
            track = [e for e in track if e[5] >= MIN_FILL]
            for i in range(1, len(track) - 1):
                if (track[i + 1][0] != track[i][0] - AUTO_T_STEP or
                        track[i - 1][0] != track[i][0] + AUTO_T_STEP):
                    continue
                growth = (track[i + 1][1] - track[i - 1][1]) / max(track[i][1], 1)
                candidates.append((growth - 0.002 * track[i][0], track[i]))

        # the best one that is also vividly coloured inside
        for _, (t, _, u, v, r, _) in sorted(candidates, key=lambda c: c[0]):
            inner = np.zeros(s.shape, np.uint8)
            cv2.circle(inner, (int(u), int(v)), max(2, int(0.7 * r)), 255, -1)
            if np.percentile(s[inner > 0], 90) >= MIN_PEAK_SAT:
                return u, v, r, t
        return None
