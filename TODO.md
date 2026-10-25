# TODO / ideas to investigate

## Mechanics and hardware

- **5:1 reduction on the arms.** About 4-4.5x torque at the arm (fewer lost steps), 5x finer
  resolution, speed / 5 (~225 deg/s arm at 10000 steps/s, enough for balancing, tight for bouncing).
  - Avoid cheap planetary gearboxes: 1-2 deg backlash makes the plate wobble.
  - Candidates: GT2 belt 16T -> 80T (printable pulley, ~no backlash, needs new arm mount),
    capstan drive (zero backlash, more work), precision planetary (< 15 arcmin, expensive).
  - Firmware: one `GEAR_RATIO` in `DEG_TO_MOTOR_STEPS`, scale max speed/accel; maybe DM556 at 1/8.
  - Check first: required torque from plate mass and accelerations (maybe 3:1 is enough).
- **Light ring.** Constant light, independent of the room. Not next to the camera under the plate
  (reflects off the acrylic into the lens); around the plate edge, angled in, or with crossed polarizers.
- **Pi screen.** Official 7" touch display on the Pi 5's second MIPI port; GUI needs bigger buttons.
- **Hide the cables.** Keep the camera ribbon away from the motor wires.

## Vision

- **Plate detection from the black dots in the white L's** as the real reference instead of the
  ball: plate height and tilt measured directly, ball size then measurable (balls of other sizes).
  Continues the paused trial on branch `experiment/plate-markers` (85 % valid detections).

## Software

- **Phone app / web page served by the Pi** (Flask + websockets, no install):
  - move the target with the finger in real time
  - draw a pattern, the ball follows it
  - minigame: tilt the phone, the plate tilts (no PID), balance the ball yourself
  - needs a UART packet for an external target; iOS motion sensors need HTTPS
- **Real bounce.** Open-loop kick on command -> detect landing from Z -> kick in sync -> tilt at
  impact. Limits: 60 fps (a 2 cm hop is 8 frames; the IMX296 is faster at lower resolution)
  and the PET-G joints under impacts.
