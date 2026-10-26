# Pong Stabilizer

A ball-on-plate balancer: a camera under a clear acrylic plate tracks a ping-pong
ball, and four stepper-driven arms tilt the plate to hold the ball still or move it
along patterns (circle, star, figure 8). It also keeps the plate at a set height
using the ball's distance from the camera.

- **Raspberry Pi 5**: camera, ball detection, GUI, sends the ball position.
- **Teensy 4.1**: PID control, target patterns, plate height, stepper motion.

Holds a still ball within 2-3 mm of the plate centre (mean) and follows moving
patterns with a mean error of 14-17 mm, at 60 fps, with no colour calibration
needed when the room lighting changes (measured in 43 min of automated tests,
see [Testing](#testing)).

---

## Contents

1. [How it works](#how-it-works)
2. [Hardware](#hardware)
3. [Repository layout](#repository-layout)
4. [Setup](#setup)
5. [Running it](#running-it)
6. [Calibration](#calibration)
7. [Ball detection](#ball-detection)
8. [Control](#control)
9. [Serial protocol](#serial-protocol)
10. [Tuning](#tuning)
11. [Logging and monitoring](#logging-and-monitoring)
12. [Testing](#testing)
13. [Troubleshooting](#troubleshooting)
14. [Known issues](#known-issues)

---

## How it works

```mermaid
flowchart LR
    subgraph Pi["Raspberry Pi 5"]
        CAM[Camera under the plate<br/>640x480 @ 60 fps] --> UND[Fisheye undistortion]
        UND --> DET["Ball detection<br/>(ball_detector.py)"]
        DET --> GATE["Plausibility check<br/>(no impossible jumps)"]
        GATE --> TRK[Position / velocity / Z<br/>in mm, plate-centred]
        TRK --> GUI[GUI + minimap + log]
    end
    subgraph Teensy["Teensy 4.1"]
        PAT[Target pattern] --> PID[PID per axis<br/>D term vs target velocity]
        PID --> MIX[Tilt mixing<br/>+ plate height offset]
        HGT[Height loop<br/>Z to 195 mm] --> MIX
        MIX --> STEP[4 x stepper control<br/>clamped to safe arm range]
        LOST[Ball gone 3 s] --> HOME[Re-home arms<br/>on the low stop]
        HOME --> STEP
    end
    TRK -- "UART 115200<br/>ball x, y, z" --> PID
    TRK -- "z" --> HGT
    PAT -- "target + plate height" --> GUI
    STEP --> ARMS[4 arms tilt and lift the plate]
    ARMS -. ball moves .-> CAM
```

1. The Pi captures a frame, removes the fisheye distortion and finds the ball. It
   converts the ball's pixel position and apparent size into **X, Y and Z in mm**
   (X/Y relative to the plate centre, Z = camera-to-ball distance) and sends them
   over UART. Detections that would mean an impossible jump are treated as "lost".
2. The Teensy compares the ball with the current target (a still point or a moving
   pattern), runs a PID per axis to get pitch and roll, and mixes them into four arm
   angles.
3. A slow height loop moves all four arms together until the ball sits 195 mm from
   the camera. That keeps the plate at the same height every run, wherever the arms
   started.
4. The Teensy sends its target and plate height back, so the Pi's GUI and log show
   exactly what the controller is doing.
5. The steppers have no encoders, so violent moves can lose steps. When the ball has
   been gone for 3 s, the arms sink slowly onto their real low stop and are zeroed
   there again.

---

## Hardware

| Part | Details |
|---|---|
| Controller | Raspberry Pi 5 (vision, GUI) + Teensy 4.1 (control) |
| Camera | Raspberry Pi Global Shutter Camera (IMX296), fisheye lens, mounted **under** the plate looking up |
| Plate | Clear, scratched acrylic octagon; flat edges ~110 mm from its centre |
| Ball | 40 mm translucent orange ping-pong ball |
| Actuation | 4 NEMA 17 steppers (1.5 A) on DM556 drivers (2.7 A RMS, 1/16 microstepping = 3200 steps/rev); 4 arms (L1 = 98.1 mm, L2 = 92 mm, L3 = 54 mm), platform radius 110 mm |
| Power | 48 V / 500 W PSU behind a 230 V master switch; 48 V → 5 V step-down for the Pi (GPIO 5 V pins); LED lighting on its own 5 V module |
| Encoders | MT6835 magnetic encoder driver included, **not used** (open loop) |
| Link | Pi `/dev/ttyAMA0` ↔ Teensy `Serial1`, 115200 baud; Teensy also on the Pi's USB (power + flashing) |

Stepper pins (dir, step) on the Teensy: M0 (7, 8) right-back, M1 (3, 4)
right-front, M2 (5, 6) left-front, M3 (9, 10) left-back.

**Arm direction:** negative steps on M0/M2 and positive steps on M1/M3 raise the
arms. The firmware handles this; it matters if you change motor wiring.

**Power:** the whole rig is switched off at the master switch, so after a power-up the
Pi shows "Reset due to low power event". That is expected, not an undervoltage problem.

The full electrical schematic (KiCad) is in `hardware/schematic/`, and the pin table
and component ratings are in the report (`docs/report/`).

---

## Repository layout

```
Pong-Stabilizer/
├── README.md                     this file
├── TODO.md                       ideas to investigate (gearbox, faster camera, bouncing, phone app...)
├── docs/
│   ├── ball-tracking-and-control.md   detailed design notes, measurements, rationale
│   └── report/                   project report (LaTeX), its figures, analysis scripts
│       └── tests/                automated test log + analyze_tests.py
├── hardware/
│   └── schematic/                KiCad project of the electrical schematic
├── pi_code/
│   ├── camera_calib/             one-time fisheye calibration
│   │   ├── 1_capture_images.py   capture checkerboard images (9x7 squares)
│   │   ├── 2_calibrate_calc.py   compute K, D -> cam_calib_fisheye.npz
│   │   ├── 3_live_undestorted.py live preview of the undistorted image
│   │   └── calibration_images/   the captured checkerboard images
│   └── ball_tracking/
│       ├── ball_detector.py      shared detection + tracking module (start here)
│       ├── 4_gui_track_w_mode.py main tracker: GUI, pattern keys, minimap, logging
│       ├── 4_gui_track_pid.py    tracker with live PID sliders
│       ├── 4_headless_track.py   tracker without GUI
│       ├── 5_test_runner.py      automated test protocol (separate from the trackers)
│       ├── 1_tune_color.py       HSV slider tuner (only for DETECTOR_MODE = "sliders")
│       ├── hsv_values.txt        HSV window for slider mode
│       ├── cam_calib_fisheye.npz camera calibration used by the trackers
│       ├── 2_track_ball.py       early standalone tracking demo (legacy)
│       └── 4_gui_track.py, 4_headless_track_pid.py   older trackers (legacy, old units)
└── teensy_code/                  PlatformIO project (teensy41, Arduino framework)
    ├── platformio.ini            depends on TeensyTimerTool
    ├── src/
    │   ├── main.cpp              start-up, patterns, edge guard, height loop, re-homing, main loop
    │   ├── inverseKinematics.cpp PID + tilt mixing
    │   ├── stepperWrapper.cpp    timer-driven stepper motion with acceleration (max 10000 steps/s)
    │   ├── uartComm.cpp          packet parser + Teensy -> Pi messages
    │   └── MT6835_encoder.cpp    encoder driver (unused)
    ├── include/                  headers; inverseKinematics.h holds most tuning constants
    ├── flash_teensy.sh           copy firmware to the Pi and flash the Teensy
    └── plot_data.py              plot x/y/z from a serial capture (out.txt)
```

---

## Setup

### Raspberry Pi

Raspberry Pi OS (Bookworm) with the camera enabled and the UART on
`/dev/ttyAMA0` enabled (serial console off).

```sh
sudo apt install python3-picamera2 python3-opencv python3-numpy python3-serial
git clone https://github.com/alexivan04/Pong-Stabilizer.git
cd Pong-Stabilizer/pi_code/ball_tracking
```

The trackers read `cam_calib_fisheye.npz` and `hsv_values.txt` from the current
directory, so run them from `pi_code/ball_tracking/`.

### Teensy firmware

Install [PlatformIO](https://platformio.org/) (CLI or the VS Code extension), then:

```sh
cd teensy_code
pio run            # builds .pio/build/teensy41/firmware.hex
```

`TeensyTimerTool` is fetched automatically (`platformio.ini`).

### Flashing

The Teensy is connected to the Pi by USB. The working method is a USB soft reboot
into the bootloader:

```sh
scp .pio/build/teensy41/firmware.hex <user>@<pi>:firmware.hex
ssh <user>@<pi> 'sudo teensy_loader_cli --mcu=TEENSY41 -s -w -v ~/firmware.hex'
```

`flash_teensy.sh <hex>` does the copy and then pulses GPIO 23 (the Teensy's program
pin) through `~/main.py` on the Pi. That pulse stopped working during testing; if
the loader sits at "Waiting for Teensy device", kill it and use the command above.

After flashing, the firmware **waits for START**, so the arms don't move until a
tracker is started.

---

## Running it

1. **Motors unpowered: rest all four arms on their mechanical low stop**, then power
   the motors. The arm position when START arrives becomes 0°.
2. On the Pi, in `pi_code/ball_tracking/`:
   ```sh
   python3 4_gui_track_w_mode.py
   ```
   It sends START. The arms lift off the stop (1 s) and balancing begins.
3. **Put the ball on right away.** The plate rises to ~195 mm from the camera while
   it balances (~25 s).
4. Keys in the tracking window:

   | Key | Action |
   |---|---|
   | `1` `2` `3` `4` | CENTER, CIRCLE, STAR, FIGURE 8 |
   | `5` | cycle all patterns, 25 s each (the start-up default) |
   | `m` | show / hide the detection mask |
   | `d` | save a snapshot (`snapshot_<time>_frame.png` + `_mask.png`) |
   | `Esc` | quit |

5. **Stopping** (`Esc`, `Ctrl-C` or `pkill -f "^python3 4_gui_track_w_mode.py"`)
   sends "ball lost" first, so the Teensy levels the plate. Stopping the Pi script
   does not cut motor power: in an emergency, switch the motors off.

Other trackers: `4_gui_track_pid.py` adds P/I/D sliders that are sent to the Teensy
live; `4_headless_track.py` runs without a window.

**Re-homing:** take the ball off and wait 3 s: the arms sink slowly onto the low stop,
are zeroed there and rise again (like at start-up). It happens once per loss, and it
corrects steps lost in violent moves. Put the ball back when the plate rises.

**Autostart:** `~/.config/autostart/ball_tracking.desktop` starts
`4_gui_track_w_mode.py` at login (enabled). There is also a systemd user service,
`ball_tracking_gui.service` (`Restart=always`, not enabled at boot): while it runs,
stop the tracker with `systemctl --user stop ball_tracking_gui.service`, because a
`pkill` is immediately restarted. Never flash the Teensy while a tracker runs.

---

## Calibration

### Camera (once per lens / mount)

1. `python3 1_capture_images.py` in `pi_code/camera_calib/`: show a 9x7-square
   checkerboard from many angles, `s` to save, `q` to finish.
2. `python3 2_calibrate_calc.py` writes `cam_calib_fisheye.npz` (K, D).
3. `python3 3_live_undestorted.py` to check the result; copy the `.npz` into
   `pi_code/ball_tracking/`.

### Ball colour: not needed

The default detector (`DETECTOR_MODE = "auto"`) needs no colour calibration. Only
if you switch to `"sliders"`, run `1_tune_color.py`. It shows the same undistorted
image and plate circle as the tracker. Tune hue and saturation, keep V wide, and
press `q` to save `hsv_values.txt`.

### Plate position

`PLATE_CENTER_PX` and `PLATE_RADIUS_MM` in `ball_detector.py` describe where the
plate is in the image. Re-measure them if the camera mount moves; a snapshot (`d`)
shows the plate clearly.

---

## Ball detection

Everything lives in `pi_code/ball_tracking/ball_detector.py`;
`create_tracker(new_K)` returns the tracker for `DETECTOR_MODE`.

**Why colour thresholds failed:** with the camera looking up through a hazy plate
at ceiling lights, skin, haze, the wooden table and a side panel all have the ball's
hue. Only saturation separates them. Brightness changes with the room light, so any
fixed HSV window breaks when the lighting changes.

**`"auto"` (default), calibration-free:** while lowering a saturation threshold step
by step, the ball shows up as a round, ball-sized blob whose outline stays the same
over many steps, a maximally stable region in the sense of MSER. The threshold is
chosen fresh every frame. A candidate must also:
- be round, judged on its convex hull so glare cutting into the edge doesn't matter;
- have its centre on the plate;
- be vividly coloured inside (rejects a dull brown patch seen through the plate).

When lost it searches the whole plate (~17 ms on a Pi 5); while tracking, only a
window around the ball (~2 ms).

**`"sliders"`, fallback:** a fixed HSV window from `hsv_values.txt`, limited to the plate.

**Plate area in mm:** the search area and plausible ball size are defined in mm and
converted to pixels using the measured plate distance, so they fit at any plate height.

**Z and smoothing:** Z comes from the ball's apparent radius. At ~200 mm the ball is
only ~13 px, so X/Y are scaled with a 15-frame median Z, and the Z sent to the Teensy
is a 5-frame median. That keeps radius noise from shaking the position.

Validated on real frames under simulated and real lighting changes: lights off, a
torch, backlight haze, warm/cool light. See the
[detailed notes](docs/ball-tracking-and-control.md#ball-detection-pi_codeball_trackingball_detectorpy).

---

## Control

All in `teensy_code/src/`; constants in `include/inverseKinematics.h`.

- **Loop:** every 5 ms. Ball position is smoothed (EMA, `ALPHA = 0.15`), compared with
  the target, and a PID per axis gives pitch/roll, limited to −12…+15°.
- **Integral:** anti-windup limit, frozen while the ball is lost, and emptied over ~1 s
  when the ball comes back after more than 0.5 s.
- **D term relative to the target:** `D · (ball velocity − target velocity)`, with the
  target smoothed like the ball. On a still target this is plain derivative on
  measurement. On moving patterns it stops braking a ball that is correctly following,
  which halved the tracking error.
- **Gains:** `Kp, Ki, Kd = 0.149, 0.112, 0.068 / UNIT_SCALE`. The Pi sends real mm;
  `UNIT_SCALE = 1.4` converts the original tuning, which was done in smaller units.
- **Patterns** (`main.cpp`): CENTER, CIRCLE (r 50 mm, 5 s), STAR (60/25 mm, 10 s),
  FIGURE 8 (60x30 mm, 6 s), or cycle all. The target moves at most 120 mm/s.
  `PATTERN_SCALE` sizes them. STEP X / STEP Y (±30 mm every 8 s) are only used by the
  test runner. All are centred on the plate centre.
- **Edge guard:** if the ball is more than 85 mm from the centre (near the bracket
  screws), the target goes back to the centre until the ball is within 40 mm again.
- **Arm range:** arm angles are degrees above the low stop. Every command is clamped
  to 5°…40°, so the arms can't grind into the stop.
- **Plate height:** all arms share a height offset. It rises to 17° (12° of tilt room),
  then follows the ball's Z to `Z_TARGET_MM = 195` (±4 mm), at up to 2°/s, between 17°
  and the 40° arm limit. It only adjusts while the ball is seen within 75 mm of the
  centre, and holds its height otherwise.
- **Start-up:** wait for START, take 0° as the current arm pose, lift to 5°, balance.
  An optional start-up tune (`STARTUP_TUNE`, off) makes the motors play a short melody.
- **Re-homing:** ball gone for `REHOME_LOST_MS` (3 s): arms sink at 6°/s to 4° below
  the counted stop, stall on the real one, and are zeroed again.

---

## Serial protocol

`0xAA | type | size | payload | XOR(type, size, payload…) | 0xBB`, 115200 baud.

| Type | Direction | Payload |
|---|---|---|
| 1 BALL_DATA | Pi → Teensy | `<Bffffff`: found, x, y, z [mm], vx, vy, vz [mm/s] (the Teensy uses the first 13 bytes) |
| 2 PID_UPDATE | Pi → Teensy | `<fff`: P, I, D |
| 3 START | Pi → Teensy | empty |
| 4 PATTERN_CHANGE | both ways | `<B`: pattern (0 CENTER, 1 CIRCLE, 2 STAR, 3 FIGURE 8, 4 cycle, 5 STEP X, 6 STEP Y) |
| 5 TARGET_POS | Teensy → Pi | `<fff`: target x, y [mm], plate height [deg above the low stop] |

Coordinates are in mm relative to the plate centre (`PLATE_CENTRE_MM` from the
camera's optical axis), with Z = camera-to-ball distance (bigger = plate higher).

---

## Tuning

| File | Constant | Default | What it does |
|---|---|---|---|
| `ball_detector.py` | `DETECTOR_MODE` | `"auto"` | `"sliders"` for the HSV fallback |
| | `PLATE_CENTER_PX`, `PLATE_RADIUS_MM` | (324, 236), 117 | Plate position in the image / allowed ball area |
| | `PLATE_CENTRE_MM` | (8.0, −1.2) | Plate centre relative to the optical axis; positions are reported from it |
| | `MAX_BALL_SPEED_MM_S`, `CONFIRM_FRAMES` | 1500, 5 | Plausibility check: faster jumps need 5 agreeing frames |
| | `MIN_PEAK_SAT` | 100 | Higher rejects more dull objects; lower if a glaring ball is missed |
| | `Z_MEDIAN_XY`, `Z_MEDIAN_OUT` | 15, 5 | More frames = steadier, slower |
| | `EXPOSURE_US`, `ANALOGUE_GAIN` | 20000, 2.0 | Fixed camera exposure |
| `inverseKinematics.h` + `ball_detector.py` | `UNIT_SCALE` | 1.4 | Higher = gentler PID (keep both files equal) |
| | `PATTERN_SCALE` | 1.0 | Pattern size and speed (keep both files equal) |
| `inverseKinematics.h` | `Z_TARGET_MM` | 195 | Plate height (camera-to-ball distance) |
| | `HEIGHT_GAIN`, `HEIGHT_RATE_DEG_S` | 0.05, 2 | Height loop speed; ~0.15 reaches the height in ~10 s |
| | `ARM_MIN_DEG`, `ARM_MAX_DEG` | 5, 40 | Arm limits above the low stop |
| | `HEIGHT_MIN_DEG`, `HEIGHT_MAX_DEG` | 17, 40 | Range of the shared plate height |
| | `EDGE_GUARD_MM`, `EDGE_RESUME_MM` | 85, 40 | Edge guard |
| | `REHOME_LOST_MS`, `REHOME_BELOW_DEG` | 3000, 4 | When and how far below the stop the arms re-home |
| `inverseKinematics.cpp` | `Kp`, `Ki`, `Kd` | 0.149/0.112/0.068 ÷ 1.4 | PID gains (or use `4_gui_track_pid.py` sliders live) |

---

## Logging and monitoring

- `TRACK_LOG=/tmp/track_run python3 4_gui_track_w_mode.py` writes `track.csv`
  (`t, found, x, y, z, vx, vy, target_x, target_y, pattern, height_deg` per frame)
  and `latest.jpg` every 2 s.
- The GUI shows status, X/Y/Z, velocities, the plate height (red with "(limit)" at the
  ends of its range), and a minimap with the Teensy's real target and the ball.
- `teensy_code/plot_data.py` plots x/y/z from a text capture (`out.txt`).

---

## Testing

`5_test_runner.py` (on the Pi, arms on the low stop, instead of a tracker) runs a
fixed protocol and tells the operator what to do: CENTER hold, 60 mm target steps on
each axis, CIRCLE / STAR / FIGURE 8 for 60 s each (each started from a ball at rest),
and five taps on the ball. A lost ball aborts the phase, which is repeated.

```sh
python3 5_test_runner.py --runs 5 --out ~/test_runs
```

`docs/report/tests/analyze_tests.py` turns the log into the report's tables and
figures (step response, tracking, disturbance rejection, screw contacts). Results
of the 25 Oct 2026 campaign (5 runs, 43 min): CENTER 2.3 mm mean error, patterns
14.2 / 14.6 / 16.5 mm (circle / star / figure 8), step overshoot ~60 %, recovery
from a tap ~3.8 s, one ball loss.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Arms grind at start-up | They weren't on the low stop when START arrived, or steps were lost; rest them on the stop and restart |
| Plate doesn't reach ~195 mm | Ball often > 75 mm from the centre (height loop pauses), or the GUI shows "(limit)" at 40°: steps were lost (the count is too high), take the ball off to re-home |
| Ball rests off-centre and the plate doesn't bring it back | A tilt offset from lost steps; take the ball off for 3 s to re-home |
| Plate tilts away after stopping | Old script version without the "ball lost" exit; restart, or send a ball-lost packet |
| Nothing moves after flashing | Expected: the firmware waits for START from a tracker |
| Pi disappears from the network | Usually switched off / rebooted; `/tmp` is then empty |
| Ball not detected | Press `m` to see the mask; check the plate circle in the GUI matches the plate; try `DETECTOR_MODE = "sliders"` |
| Flash hangs at "Waiting for Teensy device" | GPIO pulse not working; kill the loader and flash with `teensy_loader_cli -s` |
| Cursor invisible on the Pi | Don't run `unclutter -idle 0` on Wayland |

---

## Known issues

- The GPIO 23 bootloader pulse in `flash_teensy.sh` / `~/main.py` no longer works;
  use the USB soft reboot.
- `checkSerial()`: `rxBuffer` is 32 bytes and the size byte isn't bounds-checked, so a
  corrupted size can overflow it.
- `flash_teensy.sh` uses `set -u`, so running it without an argument shows "unbound
  variable" instead of the usage line.
- `4_gui_track.py` and `4_headless_track_pid.py` use the old detection and units, so
  the current gains make them too gentle. `4_gui_track_pid.py` doesn't yet read the
  Teensy's target / height packet.
- The upper arm limit (40°) is an estimate, not a measured mechanical limit.
- Open loop: lost steps are only corrected by re-homing when the ball is gone. MT6835
  encoders are supported in the code but not mounted, and their pin definitions clash
  with the UART (pin 0) and motor M3 (pin 10). A 5:1 gearbox is being considered
  instead (`TODO.md`).
- The re-homing presses the arms briefly against the low stop (slowly); it has not
  been tested much on the rig yet.

More detail, measurements and the reasoning behind each change:
[docs/ball-tracking-and-control.md](docs/ball-tracking-and-control.md).
