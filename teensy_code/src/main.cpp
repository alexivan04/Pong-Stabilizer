#define USING_MAKEFILE
#include "core_pins.h"
#include <Arduino.h>
#include <SPI.h>
#include "stepperWrapper.h"
#include "uartComm.h"
#include "inverseKinematics.h"

#define CS_ENC           0
#define CAL_ENC          12
#define ENC_A            10
#define ENC_B            11

stepperWrapper stepper_1(7, 8, NULL);  // M0: Dreapta-Spate
stepperWrapper stepper_2(3, 4, NULL);  // M1: Dreapta-Față
stepperWrapper stepper_3(5, 6, NULL);  // M2: Stânga-Față
stepperWrapper stepper_4(9, 10, NULL); // M3: Stânga-Spate

BallData ballData;
PIDValues pidValues;
boolean isStarted;
volatile float motorAngles[4];

// Pattern change requested by Pi via UART
uint8_t pendingPattern     = 0;
bool    pendingPatternFlag = false;

extern volatile float Kp;
extern volatile float Ki;
extern volatile float Kd; 

// ==========================================
// --- TRAJECTORY GENERATOR ---
// ==========================================
enum PatternType {
    PATTERN_CENTER    = 0,
    PATTERN_CIRCLE    = 1,
    PATTERN_STAR      = 2,
    PATTERN_FIGURE8   = 3,
    PATTERN_CYCLE_ALL = 4
};

PatternType currentPattern = PATTERN_CYCLE_ALL;

void getPatternTarget(PatternType pt, float t, float &tx, float &ty) {
    switch(pt) {
        case PATTERN_CENTER:
            tx = 0.0f; ty = 0.0f;
            break;

        case PATTERN_CIRCLE: {
            float radius = 50.0f;
            float period = 5.0f;
            float omega  = (2.0f * PI) / period;
            tx = radius * cos(omega * t);
            ty = radius * sin(omega * t);
            break;
        }

        case PATTERN_STAR: {
            const int num_points   = 10;
            float     outer_R      = 60.0f;
            float     inner_R      = 25.0f;
            float     period       = 10.0f;
            float     mod_t        = fmod(t, period);
            float     segment_time = period / num_points;
            int       idx          = mod_t / segment_time;
            float     progress     = (mod_t - (idx * segment_time)) / segment_time;
            int       next_idx     = (idx + 1) % num_points;

            auto get_pt = [&](int i, float &px, float &py) {
                float angle = (PI / 2.0f) + i * (PI / 5.0f);
                float r     = (i % 2 == 0) ? outer_R : inner_R;
                px = r * cos(angle);
                py = r * sin(angle);
            };

            float x1, y1, x2, y2;
            get_pt(idx,      x1, y1);
            get_pt(next_idx, x2, y2);
            tx = x1 + (x2 - x1) * progress;
            ty = y1 + (y2 - y1) * progress;
            break;
        }

        case PATTERN_FIGURE8: {
            float radius_x = 60.0f;
            float radius_y = 30.0f;
            float period   = 6.0f;
            float omega    = (2.0f * PI) / period;
            tx = radius_x * sin(omega * t);
            ty = radius_y * sin(2.0f * omega * t);
            break;
        }

        default:
            tx = 0.0f; ty = 0.0f;
            break;
    }
}

void getTrajectoryTarget(float &tx, float &ty) {
    if (!ballData.is_found) {
        tx = 0.0f; ty = 0.0f;
        return;
    }

    float t = millis() / 1000.0f;

    PatternType currType = currentPattern;

    // CYCLE_ALL: switch sub-pattern every 25 s, notify Pi on change
    if (currentPattern == PATTERN_CYCLE_ALL) {
        const float   modeDuration    = 25.0f;
        int           currentModeIndex = (int)(t / modeDuration);
        PatternType   newType          = static_cast<PatternType>(currentModeIndex % 4);

        // Detect transition and notify Pi
        static int    lastModeIndex = -1;
        if (currentModeIndex != lastModeIndex) {
            lastModeIndex = currentModeIndex;
            // Send the actual sub-pattern index (0-3), NOT 4
            sendPatternChange((uint8_t)newType);
        }

        currType = newType;
    }

    float raw_tx, raw_ty;
    getPatternTarget(currType, t, raw_tx, raw_ty);

    // --- TARGET RATE LIMITER ---
    static float    current_tx = 0.0f;
    static float    current_ty = 0.0f;
    static uint32_t lastTime   = millis();

    uint32_t now = millis();
    float    dt  = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;

    float dx   = raw_tx - current_tx;
    float dy   = raw_ty - current_ty;
    float dist = sqrt(dx * dx + dy * dy);

    float max_speed = 120.0f;
    float max_step  = max_speed * dt;

    if (dist > max_step) {
        current_tx += (dx / dist) * max_step;
        current_ty += (dy / dist) * max_step;
    } else {
        current_tx = raw_tx;
        current_ty = raw_ty;
    }

    tx = current_tx;
    ty = current_ty;
}
// ==========================================

void stepperDelay(uint32_t waitTime_ms) {
    uint32_t startTime  = millis();
    uint32_t lastUpdate = 0;

    while (millis() - startTime < waitTime_ms) {
        if (millis() - lastUpdate >= 5) {
            lastUpdate = millis();
            stepper_1.update();
            stepper_2.update();
            stepper_3.update();
            stepper_4.update();
        }
    }
}

void runMaxAngleTest() {
    stepper_1.setCurrentPositionInSteps(round( DEG_TO_MOTOR_STEPS(0)));
    stepper_2.setCurrentPositionInSteps(round(-DEG_TO_MOTOR_STEPS(0)));
    stepper_3.setCurrentPositionInSteps(round( DEG_TO_MOTOR_STEPS(0)));
    stepper_4.setCurrentPositionInSteps(round(-DEG_TO_MOTOR_STEPS(0)));

    for (int i = 0; i < 3; i++) {
        stepper_1.setTargetStep( DEG_TO_MOTOR_STEPS(MIN_ANGLE));
        stepper_2.setTargetStep(-DEG_TO_MOTOR_STEPS(MIN_ANGLE));
        stepper_3.setTargetStep( DEG_TO_MOTOR_STEPS(MIN_ANGLE));
        stepper_4.setTargetStep(-DEG_TO_MOTOR_STEPS(MIN_ANGLE));
        stepperDelay(1000);

        stepper_1.setTargetStep( DEG_TO_MOTOR_STEPS(MAX_ANGLE));
        stepper_2.setTargetStep(-DEG_TO_MOTOR_STEPS(MAX_ANGLE));
        stepper_3.setTargetStep( DEG_TO_MOTOR_STEPS(MAX_ANGLE));
        stepper_4.setTargetStep(-DEG_TO_MOTOR_STEPS(MAX_ANGLE));
        stepperDelay(1000);
    }
}

void runPongLoop() {
    ballData.is_found = 0;
    ballData.x = 0;
    ballData.y = 0;

    float start_angle = calculateArmAngle(0, 0.0f, 0.0f);
    stepper_1.setCurrentPositionInSteps(round( DEG_TO_MOTOR_STEPS(start_angle)));
    stepper_2.setCurrentPositionInSteps(round(-DEG_TO_MOTOR_STEPS(start_angle)));
    stepper_3.setCurrentPositionInSteps(round( DEG_TO_MOTOR_STEPS(start_angle)));
    stepper_4.setCurrentPositionInSteps(round(-DEG_TO_MOTOR_STEPS(start_angle)));

    // Notify Pi of the initial pattern immediately
    if (currentPattern == PATTERN_CYCLE_ALL) {
        sendPatternChange((uint8_t)PATTERN_CENTER);  // first sub-pattern at t=0
    } else {
        sendPatternChange((uint8_t)currentPattern);
    }

    uint32_t lastUpdate = 0;

    while (true) {
        checkSerial();

        // Apply pattern change requested by Pi
        if (pendingPatternFlag) {
            currentPattern     = static_cast<PatternType>(pendingPattern);
            pendingPatternFlag = false;
            // Echo back the active sub-pattern so Pi minimap updates
            if (currentPattern == PATTERN_CYCLE_ALL) {
                // Will echo on first cycle tick; send CENTER as initial state
                sendPatternChange((uint8_t)PATTERN_CENTER);
            } else {
                sendPatternChange((uint8_t)currentPattern);
            }
        }

        if (millis() - lastUpdate >= 5) {
            lastUpdate = millis();

            Kp = pidValues.P;
            Ki = pidValues.I;
            Kd = pidValues.D;

            float targetX = 0.0f;
            float targetY = 0.0f;
            getTrajectoryTarget(targetX, targetY);

            computeMotorAngles(targetX, targetY, motorAngles);

            stepper_1.setTargetStep( DEG_TO_MOTOR_STEPS(motorAngles[0]));
            stepper_2.setTargetStep(-DEG_TO_MOTOR_STEPS(motorAngles[1]));
            stepper_3.setTargetStep( DEG_TO_MOTOR_STEPS(motorAngles[2]));
            stepper_4.setTargetStep(-DEG_TO_MOTOR_STEPS(motorAngles[3]));

            stepper_1.update();
            stepper_2.update();
            stepper_3.update();
            stepper_4.update();
        }
    }
}

int main() {
    Serial.begin(115200);
    Serial1.begin(115200);
    delay(900);
    while (!isStarted);

    stepper_1.begin();
    stepper_2.begin();
    stepper_3.begin();
    stepper_4.begin();

    pidValues.P = Kp;
    pidValues.I = Ki;
    pidValues.D = Kd;

    runMaxAngleTest();
    stepperDelay(2500);
    runPongLoop();
}