#include "inverseKinematics.h"
#include "uartComm.h" 

volatile float Kp = 0.149f;
volatile float Ki = 0.112f;
volatile float Kd = 0.068f; 

float errX_int = 0, lastErrX = 0;
float errY_int = 0, lastErrY = 0;

float filteredX = 0;
float filteredY = 0;
const float ALPHA = 0.15; 

// Derivative Smoothing Variables
float smoothedDerivX = 0;
float smoothedDerivY = 0;

// PLATE BOUNDARY LIMITS (in mm)
const float PLATE_LIMIT_X = 180.0f; 
const float PLATE_LIMIT_Y = 180.0f;

// We only use this function once now to find the perfect "Zero" position.
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

    float errorX = filteredX - targetX;
    float errorY = filteredY - targetY;

    // EXACT TIMING
    static uint32_t lastTime = 0;
    uint32_t now = millis();
    float dt = (now - lastTime) / 1000.0f; 
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; 
    lastTime = now;

    // --- PID NOW OUTPUTS DIRECT DEGREE OFFSETS ---
    
    // X Axis PID (Roll / Left-Right Tilt)
    errX_int = constrain(errX_int + (errorX * dt), -50.0f, 50.0f);
    float rawDerivX = (errorX - lastErrX) / dt;
    smoothedDerivX = (0.2f * rawDerivX) + (0.8f * smoothedDerivX);
    float roll_deg = (pidValues.P * errorX) + (pidValues.I * errX_int) + (pidValues.D * smoothedDerivX);
    lastErrX = errorX;

    // Y Axis PID (Pitch / Front-Back Tilt)
    errY_int = constrain(errY_int + (errorY * dt), -50.0f, 50.0f);
    float rawDerivY = (errorY - lastErrY) / dt;
    smoothedDerivY = (0.2f * rawDerivY) + (0.8f * smoothedDerivY);
    float pitch_deg = (pidValues.P * errorY) + (pidValues.I * errY_int) + (pidValues.D * smoothedDerivY);
    lastErrY = errorY;

    // Constrain the maximum tilt to 15 degrees to prevent mechanical crashes
    pitch_deg = constrain(pitch_deg, -10.0f, 15.0f) * 1.0f;
    roll_deg = constrain(roll_deg, -10.0f, 15.0f) * -1.0f;

    // --- LINEAR DEGREE MIXING (The Magic Fix) ---
    // Get the exact mechanical "Flat" angle (usually around ~135 degrees)
    float base_angle = calculateArmAngle(0, 0.0f, 0.0f);

    // Apply algebraic symmetry. 
    // Notice how M0 is the exact mathematical inverse of M2.
    // Notice how M1 is the exact mathematical inverse of M3.
    // This forces the platform into a perfect parallelogram, making binding impossible.
    
    motorAngles[0] = base_angle - pitch_deg + roll_deg; // M0: Right-Back
    motorAngles[1] = base_angle + pitch_deg + roll_deg; // M1: Right-Front
    motorAngles[2] = base_angle + pitch_deg - roll_deg; // M2: Left-Front
    motorAngles[3] = base_angle - pitch_deg - roll_deg; // M3: Left-Back
}