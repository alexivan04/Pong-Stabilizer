# TODO / ideas to investigate

Ordered from most urgent and needed to least urgent and hardest.

## 1. Needed next

- **5:1 reduction on the arms.** About 4-4.5x torque at the arm (fewer lost steps), 5x finer
  resolution, speed / 5 (~225 deg/s arm at 10000 steps/s, enough for balancing, tight for bouncing).
  - Avoid cheap planetary gearboxes: 1-2 deg backlash makes the plate wobble.
  - Candidates: GT2 belt 16T -> 80T (printable pulley, ~no backlash, needs new arm mount),
    capstan drive (zero backlash, more work), precision planetary (< 15 arcmin, expensive).
  - Firmware: one `GEAR_RATIO` in `DEG_TO_MOTOR_STEPS`, scale max speed/accel; maybe DM556 at 1/8.
  - Check first: required torque from plate mass and accelerations (maybe 3:1 is enough).
  - **Candidate: StepperOnline MGL17-G5** ($13.75 each): spur planetary 5:1, backlash <= 30 arcmin
    (0.5 deg at the arm, ~0.45 deg plate tilt), 90 % efficiency, 9 Nm max, 1000 rpm rated / 2000 max,
    input 5 mm shaft 21.5-25 mm long, output 8 mm keyway, 55 mm long.
    https://www.omc-stepperonline.com/mgl-series-planetary-gearbox-gear-ratio-5-1-backlash-30arc-min-for-5mm-shaft-standard-nema-17-stepper-motor-mgl17-g5
    - Backlash is OK for balancing: the plate's weight preloads all arms downwards, so the teeth stay
      on one side.
    - Before ordering: measure the Robofun motor shaft length (needs 21.5-25 mm); 55 mm extra
      behind each arm in the CAD; new arm hub for 8 mm + keyway.
  - For bouncing it helps: a 5 cm hop (~1 m/s over 20 mm, ~2.5 g, plate+arms ~0.6 kg assumed)
    needs ~0.5 Nm per arm, more than the bare motor holds (~0.4 Nm) but only ~0.11 Nm with 5:1.
    ~580 deg/s at the arm = ~480 rpm motor = ~26000 steps/s at 1/16, so set the DM556 to 1/8.
    The backlash is crossed only after the ball has left (torque reverses when the plate decelerates
    faster than g): a knock per bounce (noise, wear), not a launch error.
- **Accelerometer on the base** (gravity reference for levelling). The camera is fixed to the base,
  so the plate can be level relative to the camera but not to gravity, and the ball rolls away.
  - Chain: plate vs gravity = plate vs camera (markers, see 2.) + camera vs base (fixed, calibrate
    once) + base vs gravity (accelerometer).
  - Static tilt only: plain accelerometer readings averaged over 1-2 s (removes motor vibration).
  - One-time calibration: a cheap MEMS sensor has 0.5-1 deg zero error and is never exactly
    parallel to the camera; measure the offset once (e.g. ball rolling test at first start-up),
    afterwards the sensor tracks changes by itself (rig moved to another table).
  - Connect to the Pi: I2C on header pins 3/5 is free (UART on 8/10, flash request on 16).
    Mount it rigidly on or next to the camera mount.
  - Use the IMU already bought (unused, not in the BOM); check its model for library/accuracy.
  - Cross-check with the PID integral: in CENTER, with the ball still, after a fresh homing, the
    average integral is the remaining tilt offset. Needs the integral logged first (e.g. in
    `TARGET_POS`); worth an auto-trim if it repeats within ~0.2 deg.
- **Pi screen.** Official 7" touch display on the Pi 5's second MIPI port; GUI needs bigger buttons.
  Check the 5V step-down budget (Pi 5 A recommended, display ~0.5 A).

## 2. Next, moderate effort

- **Light ring.** Constant light, independent of the room (and needed for a faster camera).
  Not next to the camera under the plate (reflects off the acrylic into the lens); around the plate
  edge, angled in, or with crossed polarizers.
- **Plate detection from the black dots in the white L's** as the real reference instead of the
  ball: plate height and tilt measured directly (detects lost steps), ball size then measurable
  (balls of other sizes). Continues the paused trial on branch `experiment/plate-markers`
  (85 % valid detections). Keep dots, not ArUco: a 4x4 tag needs ~37 mm at ~0.65 px/mm (only
  ~20 mm between the screws) and many colour changes to print. Improve contrast instead: white L
  with a black dot printed into the arm, as large as the space allows.
- **Faster camera (IMX296 row crop).** The sensor reads row by row, so fewer rows should give a
  higher frame rate (estimate, unverified: ~540 rows ~120 fps). Not in the official Raspberry Pi
  docs; community method from the Raspberry Pi forum: crop the sensor with `media-ctl` before the
  camera starts. The plate is ~450 px tall on the sensor. Measure the real fps first.
  Forum reports (2023-24): 720x240 crop ~250 fps on a Pi 5, 128x96 > 2000 fps.
  - https://forums.raspberrypi.com/viewtopic.php?t=348642 (and &start=25)
  - https://gist.github.com/Hermann-SW/e6049fe1a24fc2b5a53c654e0e9f6b9c (crop tool)
  - https://forums.raspberrypi.com/viewtopic.php?p=2301171
  - https://forums.raspberrypi.com/viewtopic.php?p=2090249#p2090249
  - https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/Global-Shutter/1.58MP-IMX296/
  - Exposure max 8.3 ms at 120 fps: half the light (needs the light ring), half the motion blur.
  - Shift the camera matrix by the crop offset (no new checkerboard needed).
  - Budget 8.3 ms per frame: tracking ~2 ms OK, full search ~17 ms skips frames.
  - Undistort only the ball's centre (`cv2.fisheye.undistortPoints`) instead of remapping the
    whole frame; use the ISP's small `lores` stream instead of converting the full image.
  - UART ~3.5 kB/s at 120 fps, under 11.5 kB/s at 115200 baud (raise the baud rate if needed).
  - Latency ~25 ms -> ~12-15 ms.
  - First test camera-only (no motors): stop the tracker, set the crop, measure fps and brightness.
- **Hide the cables.** Keep the camera ribbon away from the motor wires.

## 3. Later

- **Phone app / web page served by the Pi** (Flask + websockets, no install):
  - move the target with the finger in real time
  - draw a pattern, the ball follows it
  - minigame: tilt the phone, the plate tilts (no PID), balance the ball yourself
  - needs a UART packet for an external target; iOS motion sensors need HTTPS
- **Hardware trigger (GS camera XTR input), official:** Raspberry Pi docs, "External Trigger on the
  Global Shutter Camera". XTR is a 1.8 V input; if Q2 is fitted, remove R11; solder XTR + GND.
  Exposure = low pulse width + 14.26 us, frame rate = pulse frequency. From 3.3 V: 1.5 kOhm in
  series, 1.8 kOhm XTR-GND. Max rate still limited by the sensor readout (test with a crop).
  The Teensy could trigger every frame, so the exposure time is known exactly in the Teensy's
  clock: precise timing for bouncing and no clock conversion between Pi and Teensy.
  - https://www.raspberrypi.com/documentation/accessories/camera.html#external-trigger-on-the-global-shutter-camera
  - https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/accessories/camera/external_trigger.adoc

## 4. Hardest

- **Real bounce.** Open-loop kick on command -> detect landing from Z -> kick in sync -> tilt at
  impact. Limits: 60 fps (a 2 cm hop is 8 frames; 16 at 120 fps) and the PET-G joints under
  impacts. Much easier with the 5:1 gearbox and the faster camera (see above).
- **Two rigs bouncing the ball to each other** (after the single-rig bounce works). Plates side by
  side (~25 cm apart): with +-12 deg tilt the ball leaves ~24 deg from vertical, needs ~1.8 m/s,
  rises ~14 cm, flies ~0.7 s (~40 frames at 60 fps). Most energy comes from the ball's own
  rebound; the plate only adds what each impact loses.
  - The two Pis share the ball state over the network (UDP); the receiver predicts the landing
    point and moves its plate there.
  - The tilt at impact decides where the ball goes next, so it needs the gravity-referenced
    levelling (markers + accelerometer).
  - Check that the receiving camera sees the incoming ball early enough (fisheye, from the side).
  - Steps: stable vertical bounce -> bounce to a chosen point on the same plate -> bounce to a
    fixed target -> ping-pong between two rigs.
