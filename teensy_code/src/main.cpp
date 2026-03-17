#define USING_MAKEFILE
#include "core_pins.h"
#include <Arduino.h>
#include <SPI.h>
// #include "MT6835_encoder.h"
#include "stepperWrapper.h"
#include "uartComm.h"
#include "inverseKinematics.h"

#define CS_ENC           0
#define CAL_ENC          12
#define ENC_A            10
#define ENC_B            11

// MT6835 encoder_1(&SPI1, CS_ENC, CAL_ENC, ENC_A, ENC_B);
stepperWrapper stepper_1(7, 8, NULL);  // M0: Dreapta-Spate
stepperWrapper stepper_2(3, 4, NULL);  // M1: Dreapta-Față
stepperWrapper stepper_3(5, 6, NULL);  // M2: Stânga-Față
stepperWrapper stepper_4(9, 10, NULL); // M3: Stânga-Spate

BallData ballData;
PIDValues pidValues;
volatile float motorAngles[4];

extern volatile float Kp;
extern volatile float Ki;
extern volatile float Kd; 

// ==========================================
// --- TRAJECTORY GENERATOR ---
// ==========================================
enum PatternType {
    PATTERN_CENTER = 0,
    PATTERN_CIRCLE = 1,
    PATTERN_STAR = 2,
    PATTERN_FIGURE8 = 3,
    PATTERN_CYCLE_ALL = 4  // New cycling mode

    
};

// ---> CHANGE THIS VARIABLE TO TEST DIFFERENT PATTERNS <---
PatternType currentPattern = PATTERN_CYCLE_ALL;
PatternType activePattern;

void getTrajectoryTarget(float &tx, float &ty) {
    // Safety feature: If ball is lost, return plate to center immediately
    if (!ballData.is_found) {
        tx = 0.0f;
        ty = 0.0f;
        return;
    }

    float t = millis() / 1000.0f; // Current time in seconds
    activePattern = currentPattern;

    // If cycle mode is selected, switch the pattern every 10 seconds
    if (currentPattern == PATTERN_CYCLE_ALL) {
        const int modeDuration = 10; // Time per mode in seconds
        activePattern = static_cast<PatternType>(((int)(t / modeDuration)) % 4);
    }

    switch(activePattern) {
        case PATTERN_CENTER:
            tx = 0.0f;
            ty = 0.0f;
            break;

        case PATTERN_CIRCLE: {
            float radius = 50.0f; // Size of the circle (mm)
            float period = 5.0f;  // Seconds to complete one loop
            float omega = (2.0f * PI) / period;
            
            tx = radius * cos(omega * t);
            ty = radius * sin(omega * t);
            break;
        }

        case PATTERN_STAR: {
            // A 5-point star made by drawing straight lines between 10 alternating points
            const int num_points = 10;
            float outer_R = 60.0f; // Tip of the star
            float inner_R = 25.0f; // Inner corners of the star
            float period = 10.0f;  // Seconds to draw the whole star

            float mod_t = fmod(t, period);
            float segment_time = period / num_points;
            
            int idx = mod_t / segment_time;
            float progress = (mod_t - (idx * segment_time)) / segment_time; // 0.0 to 1.0 mapping
            
            int next_idx = (idx + 1) % num_points;
            
            // Helper lambda to calculate X/Y for a specific star point
            auto get_pt = [&](int i, float &px, float &py) {
                // Offset by PI/2 so the star points upwards
                float angle = (PI / 2.0f) + i * (PI / 5.0f); 
                float r = (i % 2 == 0) ? outer_R : inner_R;
                px = r * cos(angle);
                py = r * sin(angle);
            };
            
            float x1, y1, x2, y2;
            get_pt(idx, x1, y1);
            get_pt(next_idx, x2, y2);
            
            // Linear interpolation (draws a perfectly straight line between points)
            tx = x1 + (x2 - x1) * progress;
            ty = y1 + (y2 - y1) * progress;
            break;
        }

        case PATTERN_FIGURE8: {
            // Lissajous curve (Infinity symbol)
            float radius_x = 60.0f;
            float radius_y = 30.0f;
            float period = 6.0f;
            float omega = (2.0f * PI) / period;
            
            tx = radius_x * sin(omega * t);
            ty = radius_y * sin(2.0f * omega * t);
            break;
        }
    }
}


void stepperDelay(uint32_t waitTime_ms) {
    uint32_t startTime = millis();
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
    stepper_1.setCurrentPositionInSteps(round(DEG_TO_MOTOR_STEPS(0)));
    stepper_2.setCurrentPositionInSteps(round(-DEG_TO_MOTOR_STEPS(0)));
    stepper_3.setCurrentPositionInSteps(round(DEG_TO_MOTOR_STEPS(0)));
    stepper_4.setCurrentPositionInSteps(round(-DEG_TO_MOTOR_STEPS(0)));

    for (int i = 0; i < 3; i++) {
        stepper_1.setTargetStep(DEG_TO_MOTOR_STEPS(MIN_ANGLE));
        stepper_2.setTargetStep(-DEG_TO_MOTOR_STEPS(MIN_ANGLE));
        stepper_3.setTargetStep(DEG_TO_MOTOR_STEPS(MIN_ANGLE));
        stepper_4.setTargetStep(-DEG_TO_MOTOR_STEPS(MIN_ANGLE));

        stepperDelay(1000);

        stepper_1.setTargetStep(DEG_TO_MOTOR_STEPS(MAX_ANGLE));
        stepper_2.setTargetStep(-DEG_TO_MOTOR_STEPS(MAX_ANGLE));
        stepper_3.setTargetStep(DEG_TO_MOTOR_STEPS(MAX_ANGLE));
        stepper_4.setTargetStep(-DEG_TO_MOTOR_STEPS(MAX_ANGLE));

        stepperDelay(1000);
    }
}

void runPongLoop() {
    ballData.is_found = 0;
    ballData.x = 0;
    ballData.y = 0;
    computeMotorAngles(0, 0, motorAngles);

    stepper_1.setCurrentPositionInSteps(round(DEG_TO_MOTOR_STEPS(motorAngles[0])));
    stepper_2.setCurrentPositionInSteps(round(-DEG_TO_MOTOR_STEPS(motorAngles[1])));
    stepper_3.setCurrentPositionInSteps(round(DEG_TO_MOTOR_STEPS(motorAngles[2])));
    stepper_4.setCurrentPositionInSteps(round(-DEG_TO_MOTOR_STEPS(motorAngles[3])));

    uint32_t lastUpdate = 0;

    while(true) {
        checkSerial();

        if (millis() - lastUpdate >= 5) {
            lastUpdate = millis();

            Kp = pidValues.P;
            Ki = pidValues.I;
            Kd = pidValues.D;
            
            // --- NEW: FETCH DYNAMIC TARGET ---
            float targetX = 0.0f;
            float targetY = 0.0f;
            getTrajectoryTarget(targetX, targetY);

            // Pass the moving target to the PID kinematics
            computeMotorAngles(targetX, targetY, motorAngles);

            stepper_1.setTargetStep(DEG_TO_MOTOR_STEPS(motorAngles[0]));
            stepper_2.setTargetStep(-DEG_TO_MOTOR_STEPS(motorAngles[1]));
            stepper_3.setTargetStep(DEG_TO_MOTOR_STEPS(motorAngles[2]));
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
    while (!Serial);

    stepper_1.begin();
    stepper_2.begin();
    stepper_3.begin();
    stepper_4.begin();

    pidValues.P = Kp;
    pidValues.I = Ki;
    pidValues.D = Kd;

    runMaxAngleTest();
    
    // Wait before starting pong loop
    stepperDelay(2500); 
    
    runPongLoop();
}