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

    // Serial.println("time_ms,is_found,ball_x,ball_y,ball_z");

    uint32_t lastUpdate = 0;

    while(true) {
        checkSerial();

        // 2. Rulează Kinematica la exact 200Hz (la fiecare 5ms)
        // am dat la 2ms. poate gasesc ceva metoda mai buna, un timer sau cv
        if (millis() - lastUpdate >= 5) {
            lastUpdate = millis();

            // pid din uart
            Kp = pidValues.P;
            Ki = pidValues.I;
            Kd = pidValues.D;
            
            computeMotorAngles(0, 0, motorAngles);

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
