#include "inverseKinematics.h"
#include "uartComm.h" 

volatile float Kp = 0.149f / UNIT_SCALE;
volatile float Ki = 0.112f / UNIT_SCALE;
volatile float Kd = 0.068f / UNIT_SCALE;

float errX_int = 0, lastErrX = 0;
float errY_int = 0, lastErrY = 0;

float filteredX = 0;
float filteredY = 0;
const float ALPHA = 0.15; 

// Derivative Smoothing Variables
float smoothedDerivX = 0;
float smoothedDerivY = 0;

// Variables for Derivative-on-Measurement
float lastFilteredX = 0;
float lastFilteredY = 0;

// Target velocity for the D term, filtered exactly like the ball so both
// velocities have the same lag and their difference is the real tracking error
float filteredTX = 0, filteredTY = 0;
float lastFilteredTX = 0, lastFilteredTY = 0;
float smoothedTargetDerivX = 0, smoothedTargetDerivY = 0;

void resetPID() {
    errX_int = 0;
    lastErrX = 0;
    errY_int = 0;
    lastErrY = 0;
    filteredX = 0;
    filteredY = 0;
    smoothedDerivX = 0;
    smoothedDerivY = 0;
    lastFilteredX = 0;
    lastFilteredY = 0;
    filteredTX = filteredTY = 0;
    lastFilteredTX = lastFilteredTY = 0;
    smoothedTargetDerivX = smoothedTargetDerivY = 0;
}

// PLATE BOUNDARY LIMITS (in mm)
const float PLATE_LIMIT_X = 180.0f * UNIT_SCALE;
const float PLATE_LIMIT_Y = 180.0f * UNIT_SCALE;

float calculateArmAngle(int i, float pitch, float roll) {
    float Zi = h0 + Rp * (sin(roll) * cos(servo_gamma[i]) + sin(pitch) * sin(servo_gamma[i]));
    float Z_target = Zi - L3;
    float X_offset = abs(Rb - Rp); 

    float D = sqrt(sq(X_offset) + sq(Z_target));
    if (D > (L1 + L2)) D = L1 + L2; 

    float gamma_angle = atan2(Z_target, X_offset);
    float cosBeta = (sq(L1) + sq(D) - sq(L2)) / (2 * L1 * D);
    float beta = acos(constrain(cosBeta, -1.0f, 1.0f));

    return (gamma_angle + beta) * 180.0f / PI;
}

void computeMotorAngles(float targetX, float targetY, volatile float *motorAngles) {
    float rawX = 0;
    float rawY = 0;

    // Ball Bounds Checking
    if (ballData.is_found) {
        if (abs(ballData.x) <= PLATE_LIMIT_X && abs(ballData.y) <= PLATE_LIMIT_Y) {
            rawX = ballData.x;
            rawY = ballData.y;
        } else {
            rawX = 0;
            rawY = 0;
        }
    }

    // EMA Filter
    filteredX = (ALPHA * rawX) + ((1.0f - ALPHA) * filteredX);
    filteredY = (ALPHA * rawY) + ((1.0f - ALPHA) * filteredY);
    filteredTX = (ALPHA * targetX) + ((1.0f - ALPHA) * filteredTX);
    filteredTY = (ALPHA * targetY) + ((1.0f - ALPHA) * filteredTY);

    float errorX = filteredX - targetX;
    float errorY = filteredY - targetY;

    // EXACT TIMING
    static uint32_t lastTime = 0;
    uint32_t now = millis();
    float dt = (now - lastTime) / 1000.0f; 
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; 
    lastTime = now;

    // --- D TERM: BALL VELOCITY RELATIVE TO THE TARGET'S ---
    // Still derivative on measurement (no kick when the pattern jumps), but a
    // ball that moves along with a moving target isn't braked any more; braking
    // it made the ball lag the circle/figure-8 by ~0.4 s and swing wide.
    // With a still target (CENTER) this is the same as before.

    // X Axis PID (Roll / Left-Right Tilt)
    errX_int = constrain(errX_int + (errorX * dt), -50.0f * UNIT_SCALE, 50.0f * UNIT_SCALE);
    float rawDerivX = (filteredX - lastFilteredX) / dt;
    smoothedDerivX = (0.2f * rawDerivX) + (0.8f * smoothedDerivX);
    float rawTargetDerivX = (filteredTX - lastFilteredTX) / dt;
    smoothedTargetDerivX = (0.2f * rawTargetDerivX) + (0.8f * smoothedTargetDerivX);
    float roll_deg = (pidValues.P * errorX) + (pidValues.I * errX_int)
                   + (pidValues.D * (smoothedDerivX - smoothedTargetDerivX));

    lastFilteredX = filteredX;
    lastFilteredTX = filteredTX;
    lastErrX = errorX;

    // Y Axis PID (Pitch / Front-Back Tilt)
    errY_int = constrain(errY_int + (errorY * dt), -50.0f * UNIT_SCALE, 50.0f * UNIT_SCALE);
    float rawDerivY = (filteredY - lastFilteredY) / dt;
    smoothedDerivY = (0.2f * rawDerivY) + (0.8f * smoothedDerivY);
    float rawTargetDerivY = (filteredTY - lastFilteredTY) / dt;
    smoothedTargetDerivY = (0.2f * rawTargetDerivY) + (0.8f * smoothedTargetDerivY);
    float pitch_deg = (pidValues.P * errorY) + (pidValues.I * errY_int)
                    + (pidValues.D * (smoothedDerivY - smoothedTargetDerivY));

    lastFilteredY = filteredY;
    lastFilteredTY = filteredTY;
    lastErrY = errorY;

    // Constrain the maximum tilt to 15 degrees to prevent mechanical crashes
    pitch_deg = constrain(pitch_deg, -12.0f, 15.0f) * 1.0f;
    roll_deg = constrain(roll_deg, -12.0f, 15.0f) * -1.0f;

    // --- LINEAR DEGREE MIXING ---
    // Tilt only, relative to the current plate height; main.cpp adds the
    // height and keeps every arm inside its safe range.
    motorAngles[0] = - pitch_deg + roll_deg; // M0: Right-Back
    motorAngles[1] = + pitch_deg + roll_deg; // M1: Right-Front
    motorAngles[2] = + pitch_deg - roll_deg; // M2: Left-Front
    motorAngles[3] = - pitch_deg - roll_deg; // M3: Left-Back
}