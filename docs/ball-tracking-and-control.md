# Ball tracking and plate control

How the Pi's ball detection and the Teensy's control work after the October 2026
changes, why they changed, how to run and tune the rig, and what is still open.

## The short version

| Area | Before | Now |
|---|---|---|
| Ball detection | HSV sliders; the brightness window had to be re-tuned whenever the room light changed | **Automatic**: no colour calibration (`DETECTOR_MODE = "auto"`); sliders kept as a fallback |
| Search area | Whole image (face, wood and a side panel were mistaken for the ball) | Only the plate, defined in mm so it fits at any plate height |
| Position units | ~1/1.4 of real mm (the ball radius was over-estimated) | Real mm; gains and distances tuned in the old units are scaled by `UNIT_SCALE = 1.4` |
| Z (camera-to-ball distance) | Recomputed per frame, jumped up to 70 mm between frames, kicking the plate | Median-smoothed; X/Y use a steadier Z |
| D term | Braked the ball even when it correctly followed a moving target | Acts on ball velocity relative to the target velocity |
| Start-up | Firmware ignored START (the compiler removed the wait); arms moved at every boot | Waits for START; arms start from the low stop |
| Plate height | Wherever the arms happened to rest at power-on (Z 145 one day, 200 the next) | Closed loop on the camera's Z, target 195 mm |
| Stopping the Pi script | Teensy kept acting on the last ball position; the I term wound the plate up | Script sends "ball lost" on exit, also on `pkill`/SIGTERM |

Measured on the rig (ball found 99.9-100 % of frames, 60 fps):

| Pattern | First run after the detection change | Final runs |
|---|---|---|
| CENTER (avg distance from target) | ~4 mm | 1.3-4 mm |
| CIRCLE | ~35 mm | 12-16 mm |
| STAR | 19-21 mm | 10-16 mm |
| FIGURE 8 | ~37 mm | 13-18 mm |

## The rig, as the camera sees it

These facts drove most of the design and weren't written down anywhere:

- The camera is **under** a clear, scratched acrylic plate, looking up at the
  ceiling lights. The ball is a translucent orange ping-pong ball, lit from
  behind, so it glows.
- Skin, plate haze, a side panel and the wooden table have the **same hue** as
  the ball. Only saturation separates them: ball S 140-225, the rest S 60-110.
  Brightness (V) is what changes with the room light, which is why the old V
  window kept needing re-tuning.
- **Bigger Z = higher plate** (the camera is below). Z is the distance from the
  camera to the ball centre, computed from the ball's apparent radius.
- The plate's flat edges are ~110 mm from its centre; it is seen centred at
  pixel (324, 236) in the undistorted image. Real focal length after
  undistortion: fx ≈ 144.5 px.
- Arm direction: **negative steps on M0/M2 and positive on M1/M3 raise the
  arms.** (The old start-up stroke of +15° drove the arms down into the low stop.)
- The arms' safe travel above the low stop isn't measured. The firmware's hard
  limit is 40°; the old start-up test went 25° above the Z ≈ 200 mm height, so
  normal running stays well inside what the rig has done before.

## Running the rig

1. **Motors unpowered, rest all four arms on their low stop**, then power the
   motors. The Teensy waits; nothing moves.
2. Start the tracker on the Pi, from `~/pi_code/ball_tracking`:
   `python3 4_gui_track_w_mode.py`
   (optionally `TRACK_LOG=/tmp/track_run python3 4_gui_track_w_mode.py` to log, see below).
   The script sends START. **The arm position at that moment is 0°.**
3. The arms lift to 5° over 1 s, then balancing starts. **The ball can go on
   straight away.** While balancing, the plate rises to its 15° minimum and then
   to Z ≈ 195 mm (about 25 s with the current gain).
4. Keys in the window: `1`-`4` patterns (CENTER, CIRCLE, STAR, FIGURE 8), `5`
   cycle all, `m` show the detection mask, `d` save a snapshot (frame + mask PNG),
   `Esc` quit.
5. **Stop** with `Esc`, `Ctrl-C` or `pkill -f "^python3 4_gui_track_w_mode.py"`.
   The script sends "ball lost" before closing, so the Teensy levels the plate.
   Stopping the Pi doesn't stop the motors: in an emergency, cut the motor power.

Autostart (`~/.config/autostart/ball_tracking.desktop`) runs
`4_gui_track_w_mode.py` but is currently **disabled** with `Hidden=true`. It no
longer starts `unclutter -idle 0`, which spun at ~54 % CPU on Wayland, hid the
cursor, and never exited, so the old autostart never actually started the tracker.

### Flashing the Teensy

The GPIO 23 "program pin" pulse used by `flash_teensy.sh` / `~/main.py` stopped
working during testing (the loader waits forever for the bootloader). The USB
soft reboot works:

```sh
pio run                                                      # in teensy_code/
scp .pio/build/teensy41/firmware.hex biban@<pi>:firmware.hex
ssh biban@<pi> 'sudo teensy_loader_cli --mcu=TEENSY41 -s -w -v ~/firmware.hex'
```

The firmware waits for START after booting, so flashing doesn't move the arms.

## Ball detection (`pi_code/ball_tracking/ball_detector.py`)

All tracking scripts use one shared module. `create_tracker(new_K)` returns the
tracker selected by `DETECTOR_MODE`; both return a `BallState` (position and
velocity in mm, Z in mm, pixel circle for drawing).

### `"auto"`: calibration-free (default)

The ball is the round, ball-sized blob on the plate whose outline stays the same
while a saturation threshold is lowered step by step, a maximally stable region
in the sense of MSER. The threshold is chosen per frame, so brightness, colour
temperature and backlight haze don't need re-tuning.

Per candidate blob, while lowering the threshold in steps of `AUTO_T_STEP` (6)
from the 99.9th saturation percentile down to `AUTO_T_MIN` (50):

- radius between `r_min` and `r_max`, derived from plausible Z (90-320 mm)
  and the 40 mm ball;
- roundness: **convex hull** area / enclosing-circle area ≥ `MIN_FILL` (0.6).
  The hull bridges the gap a glare spot cuts into the ball's edge;
- centre on the plate (see below).

The candidate with the least area growth between neighbouring thresholds wins,
provided its inner 90th-percentile saturation is ≥ `MIN_PEAK_SAT` (100). That last
check rejects a dull brown patch seen through the plate's corner (S 72-74) that is
round and stable; the real ball is ≥ 113 in every tested condition.

Speed: searching the whole plate costs ~17 ms on the Pi 5, so it only runs while
the ball is lost. While tracking, only a window around the ball and a threshold
band of ±30 around the last value are searched, ~2 ms (sliders: 1.5 ms).

How it was validated: 11 real snapshots plus simulated darker/brighter, warmer/
cooler light and backlight haze, then live runs with lights off and a torch. Fixed
thresholds tuned on the same frames failed under warm light (0/11) and haze
(6/11). The automatic mode found 76/77, and stayed at 100 % in the live lighting test.

### `"sliders"`: fallback

The original HSV window from `hsv_values.txt` (tune with `1_tune_color.py`,
which now shows the same undistorted image and plate circle as the tracker). New
defaults H 98-116, **S ≥ 145**, V 40-255. Keep V wide and tune H/S.

### Plate area in mm

`PLATE_CENTER_PX`, `PLATE_RADIUS_MM` (117, the ball centre must be within this),
`ROI_MARGIN_MM` (22, pixels searched beyond it so an edge ball stays whole). They
are converted to pixels with the camera-to-plate distance, learned from the
ball's Z (`PLATE_Z_ALPHA`), starting from a generous `PLATE_Z_START_MM` (125).

### Z, X and Y

`Z = fx·D / (2r) · sqrt(1 + (d/fx)²)` as before. At ~200 mm the ball is only
~13 px, so 1 px of radius error is ~7 % of Z, and X/Y (scaled by Z) jumped with it.
Now:

- X/Y are scaled by the median Z of the last `Z_MEDIAN_XY` (15) frames;
- the Z sent to the Teensy is the median of the last `Z_MEDIAN_OUT` (5) frames.

Position jumps between frames (99th percentile) went from 7.6 to ~4 mm. Z itself
is still sent every frame for the height loop and later bouncing. Checked on logs
at two plate heights: Z stays within ~4 mm of its centre value out to 80 mm from
the centre, so no position correction is applied.

## Units and scaling

The old slider settings over-estimated the ball radius (haze and fingers merged
in), so the old "mm" were ~1/1.4 of real mm (median ratio over the snapshots, range
1.1-1.7). Real mm are kept and everything tuned in the old units is scaled:

- `UNIT_SCALE = 1.4` (in `inverseKinematics.h` **and** `ball_detector.py`):
  Kp, Ki, Kd ÷ 1.4 (0.106 / 0.080 / 0.049); plate limit and integral limit × 1.4;
  PID-GUI slider defaults ÷ 1.4.
- `PATTERN_SCALE = 1.0` (in `inverseKinematics.h` **and** `ball_detector.py`):
  pattern size and target speed, a design choice and not a unit conversion. At
  1.4 the star and figure-8 reached 84 mm and the ball ran off the plate. The GUI
  minimap uses the same value.

## Control changes (Teensy firmware)

### D term relative to the target

`D * (ball velocity − target velocity)`. The target goes through the same
smoothing as the ball (`ALPHA` EMA, then the 0.2 velocity EMA), so both velocities
have the same lag. Still derivative on measurement (no kick when a pattern jumps).
With a still target (CENTER) it's identical to before. The tracking error on
moving patterns roughly halved (circle ~35 → 13-19 mm). A simulation of the loop
predicted the same direction.

### START wait (bug fix)

`while (!isStarted);` with a non-volatile flag and no serial read inside: the
compiler removed the loop, so the firmware started the moment it booted and the
START packet never mattered. Now `volatile` and `while (!isStarted) checkSerial();`.
This was confirmed in the disassembly.

### Arm range and start-up

- Arm angles are in degrees above the low stop. `setArms()` clamps every command
  to `[ARM_MIN_DEG, ARM_MAX_DEG]` = [5°, 40°], so the arms can't grind into the stop.
  Previously full tilt could command an arm up to 30° below neutral.
- Start-up (`initArms()`): set 0° at START, ramp to 5° over 1 s. The old
  three-stroke ±15° test was removed: it ground into the stop, lost steps, and
  threw a ball that was put on early.

### Plate height loop (`updatePlateHeight()`)

All arms share one height offset. After rising to `HEIGHT_MIN_DEG` (15°, leaving
10° of tilt room below) at 2°/s, it moves at `HEIGHT_GAIN` (0.05 °/s per mm,
capped at 2°/s) until the smoothed Z is within ±4 mm of `Z_TARGET_MM` (195), inside
[15°, 30°]. It only acts while the ball is seen at a believable Z and within 75 mm
of the centre; without a ball it holds its height. The tilt from the PID is
subtracted from the height for each arm.

Measured: 1° of arm ≈ 1.6 mm of Z. The plate went 5° → 25.5° and reached Z 191 mm
after ~25 s, then held 25.6-26.1° (Z 190-196 mm).

## Serial protocol (Pi ↔ Teensy, 115200 baud)

Frame: `0xAA | type | size | payload | XOR(type, size, payload) | 0xBB`

| Type | Direction | Payload |
|---|---|---|
| 1 BALL_DATA | Pi → Teensy | `<Bffffff`: found, x, y, z (mm), vx, vy, vz (mm/s). The Teensy reads the first 13 bytes |
| 2 PID_UPDATE | Pi → Teensy | `<fff` P, I, D |
| 3 START | Pi → Teensy | empty |
| 4 PATTERN_CHANGE | both | `<B` pattern index |
| 5 TARGET_POS | Teensy → Pi | `<fff`: target x, target y (mm), **plate height (deg above the low stop)**; was 8 bytes without the height |

## Monitoring and logs

- `TRACK_LOG=<dir>` makes `4_gui_track_w_mode.py` write `<dir>/track.csv`
  (`t,found,x,y,z,vx,vy,target_x,target_y,pattern,height_deg`, every frame) and
  `<dir>/latest.jpg` every 2 s.
- The GUI shows `Plate: <deg>`, in red with `(limit)` at 15° or 30°. The minimap's
  target dot is now the Teensy's real target; it was recomputed from the Pi's
  clock and could be on the wrong side of the figure-8.
- The `pattern` column follows PATTERN_CHANGE packets and can lag the Teensy;
  the target columns are exact.

## Tuning knobs

| Where | Name | Value | Effect |
|---|---|---|---|
| `ball_detector.py` | `DETECTOR_MODE` | `"auto"` | `"sliders"` for the HSV fallback |
| | `PLATE_CENTER_PX`, `PLATE_RADIUS_MM` | (324, 236), 117 | Re-measure if the camera mount moves |
| | `MIN_PEAK_SAT` | 100 | Raise if dull objects get detected; lower if the ball under strong glare is missed |
| | `Z_MEDIAN_XY` / `Z_MEDIAN_OUT` | 15 / 5 | More frames = steadier, slower |
| both | `UNIT_SCALE` | 1.4 | Higher = gentler PID |
| both | `PATTERN_SCALE` | 1.0 | Pattern size and speed |
| `inverseKinematics.h` | `Z_TARGET_MM` | 195 | Plate height |
| | `HEIGHT_GAIN` | 0.05 | ~0.15 would reach the height in ~10 s instead of ~25 s |
| | `ARM_MIN_DEG` / `ARM_MAX_DEG` | 5 / 40 | Arm limits above the low stop |

## Known issues and next steps

- **Pi undervoltage**: the desktop showed "Reset due to low power event" and the
  Pi became unreachable several times. Check the supply (Pi 5: 5 V / 5 A, ideally
  separate from the motors); `vcgencmd get_throttled` shows the state.
- **GPIO 23 flash pulse** no longer enters the bootloader; use the USB soft reboot above.
- **`checkSerial()` buffer overflow**: `rxBuffer` is 32 bytes and the size byte
  isn't checked, so a corrupted size > 28 writes past the buffer.
- **`flash_teensy.sh`** has `set -u`, so running it without an argument fails
  with "unbound variable" instead of printing the usage message.
- **Lost steps** only show as an offset near the low stop; the Z loop still keeps
  the height right. A soft "cranking" can remain at the very start.
- `4_gui_track.py` and `4_headless_track_pid.py` still have their own old
  detection and positions in the old units, so the scaled gains make them too
  gentle. `4_gui_track_pid.py` doesn't read the Teensy's target/height packet yet.
- The upper arm limit (40°) is an estimate, not measured.
