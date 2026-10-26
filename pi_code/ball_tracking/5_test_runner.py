#!/usr/bin/env python3
"""Automated test protocol for the report: runs every test phase on its own,
logs each camera frame with its phase, and tells the operator what to do.

Separate from the normal trackers (4_*.py), which are not changed by it.
Needs the Teensy firmware with the STEP_X / STEP_Y test patterns (5, 6).

Usage (on the Pi, arms resting on the low stop):
    python3 5_test_runner.py [--runs 5] [--out ~/test_runs] [--no-taps]

Per run (about 8.5 min):
    SETTLE 25 s, CENTER 60 s, STEP_X 64 s, STEP_Y 64 s,
    then CIRCLE, STAR and FIGURE 8 for 60 s each, every test preceded by
    10 s of CENTER so it always starts from a ball at rest, and finally
    5 taps on the ball (DISTURB), one every 8 s.
If the ball is lost for more than 1 s the current phase is aborted, the
operator puts the ball back, and the phase is repeated.
"""
import argparse
import math
import os
import signal
import struct
import sys
import time

import cv2
import serial

from ball_detector import create_tracker, load_undistort, start_camera

START_BYTE, END_BYTE = 0xAA, 0xBB
PACKET_TYPE_BALL_DATA      = 1
PACKET_TYPE_START          = 3
PACKET_TYPE_PATTERN_CHANGE = 4
PACKET_TYPE_TARGET_POS     = 5
PATTERNS = {"CENTER": 0, "CIRCLE": 1, "STAR": 2, "FIGURE 8": 3, "STEP_X": 5, "STEP_Y": 6}

LOST_ABORT_S = 1.0     # ball missing this long: phase aborted
TAPS         = 5
TAP_EVERY_S  = 8.0


def protocol(taps):
    """(phase name, pattern, duration s), for one run."""
    steps = [("SETTLE", "CENTER", 25.0), ("CENTER", "CENTER", 60.0)]
    for name in ["STEP_X", "STEP_Y", "CIRCLE", "STAR", "FIGURE 8"]:
        steps += [("REST", "CENTER", 10.0), (name, name, 64.0 if name.startswith("STEP") else 60.0)]
    if taps:
        steps += [("REST", "CENTER", 10.0), ("DISTURB", "CENTER", TAPS * TAP_EVERY_S + 4.0)]
    return steps


class Teensy:
    def __init__(self, port="/dev/ttyAMA0", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0)
        self.buf, self.state = bytearray(), 0
        self.target = (0.0, 0.0)
        self.height = float("nan")
        self.pattern = -1

    def send(self, ptype, payload=b""):
        csum = ptype ^ len(payload)
        for b in payload:
            csum ^= b
        self.ser.write(struct.pack("<BBB", START_BYTE, ptype, len(payload)) + payload
                       + struct.pack("<BB", csum, END_BYTE))

    def ball(self, found, x, y, z, vx, vy, vz):
        self.send(PACKET_TYPE_BALL_DATA, struct.pack("<Bffffff", found, x, y, z, vx, vy, vz))

    def set_pattern(self, idx):
        self.send(PACKET_TYPE_PATTERN_CHANGE, struct.pack("<B", idx))

    def poll(self):
        while self.ser.in_waiting > 0:
            b = self.ser.read(1)[0]
            if self.state == 0:
                if b == START_BYTE:
                    self.buf, self.state = bytearray(), 1
                continue
            self.buf.append(b)
            if len(self.buf) < 2 or len(self.buf) < self.buf[1] + 4:
                continue
            ptype, size = self.buf[0], self.buf[1]
            payload = bytes(self.buf[2:2 + size])
            csum = ptype ^ size
            for c in payload:
                csum ^= c
            if self.buf[-1] == END_BYTE and self.buf[-2] == csum:
                if ptype == PACKET_TYPE_TARGET_POS and size == 12:
                    tx, ty, self.height = struct.unpack("<fff", payload)
                    self.target = (tx, ty)
                elif ptype == PACKET_TYPE_PATTERN_CHANGE and size >= 1:
                    self.pattern = payload[0]
            self.state = 0

    def close(self):
        # ball lost, so the Teensy doesn't keep acting on the last position
        for _ in range(3):
            self.ball(0, 0, 0, 0, 0, 0, 0)
        self.ser.flush()
        self.ser.close()


def say(msg):
    print(f"\n>>> {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--out", default=os.path.expanduser("~/test_runs"))
    ap.add_argument("--no-taps", action="store_true")
    args = ap.parse_args()

    out = os.path.join(args.out, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    log = open(os.path.join(out, "test.csv"), "w", buffering=1)
    log.write("t,t_exposure,found,x,y,z,vx,vy,target_x,target_y,pattern,height_deg,run,phase,event\n")

    def on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)

    map1, map2, new_K = load_undistort()
    cam = start_camera()
    tracker = create_tracker(new_K)
    teensy = Teensy()
    teensy.send(PACKET_TYPE_START)
    # offset from the sensor clock (CLOCK_BOOTTIME, ns) to time.time()
    boot_to_epoch = time.time() - time.clock_gettime(time.CLOCK_BOOTTIME)

    state = dict(started=False, last_seen=time.time(), last_status=0.0, last_jpg=0.0)

    def frame_step(run, phase, event=""):
        """Grab, track, send and log one frame. Returns the ball state."""
        teensy.poll()
        req = cam.capture_request()
        try:
            raw = req.make_array("main")
            ts = req.get_metadata().get("SensorTimestamp")
        finally:
            req.release()
        t_exp = ts / 1e9 + boot_to_epoch if ts else float("nan")
        frame = cv2.remap(raw, map1, map2, cv2.INTER_LINEAR)
        ball = tracker.update(frame)      # positions relative to the plate centre
        bx, by = ball.x, ball.y
        now = time.time()
        if ball.found and not state["started"]:
            teensy.send(PACKET_TYPE_START)
            state["started"] = True
        if state["started"]:
            teensy.ball(1 if ball.found else 0, bx, by, ball.z, ball.vx, ball.vy, ball.vz)
        if ball.found:
            state["last_seen"] = now
        tx, ty = teensy.target
        log.write(f"{now:.4f},{t_exp:.4f},{int(ball.found)},{bx:.2f},{by:.2f},{ball.z:.1f},"
                  f"{ball.vx:.0f},{ball.vy:.0f},{tx:.2f},{ty:.2f},{teensy.pattern},{teensy.height:.2f},"
                  f"{run},{phase},{event}\n")
        if now - state["last_jpg"] >= 2.0:
            state["last_jpg"] = now
            if ball.found:
                cv2.circle(frame, (int(ball.u), int(ball.v)), int(ball.r), (0, 255, 0), 2)
            cv2.imwrite(os.path.join(out, "latest.jpg"), frame)
        return ball, now

    def status(run, phase, remaining, ball):
        now = time.time()
        if now - state["last_status"] < 0.5:
            return
        state["last_status"] = now
        sys.stdout.write(f"\r[run {run}] {phase:9s} {remaining:5.1f} s left | "
                         f"{'FOUND' if ball.found else 'LOST '} x {ball.x:6.1f} y {ball.y:6.1f} "
                         f"z {ball.z:5.1f} | plate {teensy.height:4.1f} deg   ")
        sys.stdout.flush()

    def wait_for_ball(run):
        teensy.set_pattern(PATTERNS["CENTER"])
        say("Put the ball on the plate (centre).")
        seen_since = None
        while True:
            ball, now = frame_step(run, "WAIT_BALL")
            status(run, "WAIT_BALL", 0.0, ball)
            if ball.found:
                seen_since = seen_since or now
                if now - seen_since > 3.0:
                    say("Ball found, continuing.")
                    return
            else:
                seen_since = None

    def run_phase(run, phase, pattern, duration):
        """True if completed, False if the ball was lost."""
        teensy.set_pattern(PATTERNS[pattern])
        t0 = time.time()
        next_tap = t0 + TAP_EVERY_S if phase == "DISTURB" else None
        taps_done, prompted = 0, set()
        if phase == "DISTURB":
            say(f"Get ready: tap the ball lightly {TAPS} times when told (one sideways nudge, then let go).")
        while True:
            event = ""
            if next_tap is not None and taps_done < TAPS:
                left = next_tap - time.time()
                for c in (3, 2, 1):
                    if left <= c and (taps_done, c) not in prompted:
                        prompted.add((taps_done, c))
                        say(f"tap {taps_done + 1} in {c}...")
                if left <= 0:
                    say(f"TAP {taps_done + 1} NOW")
                    event = f"tap{taps_done + 1}"
                    taps_done += 1
                    next_tap += TAP_EVERY_S
            ball, now = frame_step(run, phase, event)
            status(run, phase, duration - (now - t0), ball)
            if now - state["last_seen"] > LOST_ABORT_S:
                log.write(f"{now:.4f},,0,,,,,,,,,,{run},{phase},ball_lost\n")
                say(f"Ball lost during {phase}: phase will be repeated.")
                return False
            if now - t0 >= duration:
                return True

    try:
        say("Arms must rest on their low stop. The Teensy lifts the plate once the ball is seen.")
        wait_for_ball(1)
        for run in range(1, args.runs + 1):
            say(f"Run {run} of {args.runs}")
            for phase, pattern, duration in protocol(not args.no_taps):
                while not run_phase(run, phase, pattern, duration):
                    wait_for_ball(run)
                    run_phase(run, "SETTLE", "CENTER", 10.0)
        say(f"All runs done. Log: {out}/test.csv")
    except KeyboardInterrupt:
        say("Stopped.")
    finally:
        teensy.set_pattern(PATTERNS["CENTER"])
        cam.stop()
        teensy.close()
        log.close()


if __name__ == "__main__":
    main()
