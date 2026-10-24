#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

// #include "TeensyTimerTool.h"
#include "uartComm.h"

// Dimensiuni mecanice (modifică valorile cu cele reale)
const float L1 = 98.1;    // Lungime braț motor (mm)
const float L2 = 92.0;    // Lungime braț intermediar (mm)
const float L3 = 54.0;    // Lungime link sub platformă (mm)
const float Rb = 129.0;   // Raza bazei (centru - ax motor) (mm)
const float Rp = 110.0;    // Raza platformei (centru - prindere) (mm)
const float h0 = 100.0;   // Înălțimea neutră (mm)

// Ball positions from the Pi are real mm since the calibration-free detector;
// the old HSV detector reported ~1/1.4 of that (it over-estimated the ball
// radius). Every gain and distance tuned in the old units is scaled by this,
// so the behaviour is unchanged. Keep in sync with UNIT_SCALE in ball_detector.py.
const float UNIT_SCALE = 1.4f;

// Size/speed of the target patterns in main.cpp, in real mm. Separate from
// UNIT_SCALE on purpose: it's a design choice, not a unit conversion. At 1.4
// the star/figure-8 reached 84 mm and the ball ran off the ~120 mm plate.
// Keep in sync with PATTERN_SCALE in ball_detector.py (GUI minimap).
const float PATTERN_SCALE = 1.0f;

// --- ARM RANGE: the arms must rest on their mechanical low stop when START
// arrives from the Pi; that pose is 0 deg (positive = up). Every arm command is kept inside
// [ARM_MIN_DEG, ARM_MAX_DEG], so the arms never grind into the low stop.
const float ARM_MIN_DEG       = 5.0f;   // stay this far above the low stop (2 deg still ground)
const float ARM_MAX_DEG       = 40.0f;  // hard limit above the low stop; the old start-up test went
                                        // 25 deg above the Z~200 mm neutral, so normal running stays below
const float ARM_LIFT_S        = 1.0f;   // start-up: lift from the stop to ARM_MIN_DEG this slowly

// --- PLATE HEIGHT: while balancing, all arms rise/sink together until the
// ball is Z_TARGET_MM from the camera (the Pi measures Z from the ball's size;
// the camera is under the plate, so raising the plate increases Z) ---
const float Z_TARGET_MM       = 195.0f; // where the plate used to sit (190-200 mm)
const float Z_DEADBAND_MM     = 4.0f;   // close enough: hold the height
const float HEIGHT_GAIN       = 0.05f;  // deg/s of arm motion per mm of height error...
const float HEIGHT_RATE_DEG_S = 2.0f;   // ...capped at this
// The shared height keeps 10 deg of tilt room on both sides; if Z_TARGET_MM
// isn't reachable inside this, the plate stops at the nearest end.
const float HEIGHT_MIN_DEG    = ARM_MIN_DEG + 10.0f;
const float HEIGHT_MAX_DEG    = ARM_MAX_DEG - 10.0f;

const float servo_gamma[4] = {5*PI/4.0f, 3*PI/4.0f, PI/4.0f, 7*PI/4.0f};
extern volatile float motorAngles[4];

float calculateArmAngle(int i, float pitch, float roll);
void computeMotorAngles(float targetX, float targetY, volatile float *motorAngles);

#endif
