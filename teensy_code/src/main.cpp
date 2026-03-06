#define USING_MAKEFILE
#include "core_pins.h"
#include <Arduino.h>
#include <SPI.h>
#include "MT6835_encoder.h"
#include "stepperWrapper.h"
#include "uartComm.h"
#include "inverseKinematics.h"

#define CS_ENC           0
#define CAL_ENC          12
#define ENC_A            10
#define ENC_B            11

MT6835 encoder_1(&SPI1, CS_ENC, CAL_ENC, ENC_A, ENC_B);
stepperWrapper stepper_1(7, 8, NULL);  // M0: Dreapta-Spate
stepperWrapper stepper_2(3, 4, NULL);  // M1: Dreapta-Față
stepperWrapper stepper_3(5, 6, NULL);  // M2: Stânga-Față
stepperWrapper stepper_4(9, 10, NULL); // M3: Stânga-Spate

BallData ballData;
volatile float motorAngles[4];

void runPongLoop() {
    // Calculăm poziția zero
    ballData.is_found = 0;
    ballData.x = 0;
    ballData.y = 0;
    computeMotorAngles(0, 0, motorAngles);

    // Sincronizare la poziția actuală pentru a nu sări brusc
    stepper_1.setCurrentPositionInSteps(round(DEG_TO_MOTOR_STEPS(motorAngles[0])));
    stepper_2.setCurrentPositionInSteps(round(-DEG_TO_MOTOR_STEPS(motorAngles[1])));
    stepper_3.setCurrentPositionInSteps(round(DEG_TO_MOTOR_STEPS(motorAngles[2])));
    stepper_4.setCurrentPositionInSteps(round(-DEG_TO_MOTOR_STEPS(motorAngles[3])));

    // Pornim controllerele motoarelor
    stepper_1.begin();
    stepper_2.begin();
    stepper_3.begin();
    stepper_4.begin();

    Serial.println("time_ms,is_found,ball_x,ball_y,ball_z");

    uint32_t lastUpdate = 0;

    while(true) {
        // 1. Citește UART constant (NON-BLOCKING)
        checkSerial();

        // 2. Rulează Kinematica la exact 200Hz (la fiecare 5ms)
        if (millis() - lastUpdate >= 5) {
            lastUpdate = millis();

            computeMotorAngles(0, 0, motorAngles);

            stepper_1.setTargetStep(DEG_TO_MOTOR_STEPS(motorAngles[0]));
            stepper_2.setTargetStep(-DEG_TO_MOTOR_STEPS(motorAngles[1]));
            stepper_3.setTargetStep(DEG_TO_MOTOR_STEPS(motorAngles[2]));
            stepper_4.setTargetStep(-DEG_TO_MOTOR_STEPS(motorAngles[3]));

            // Accelerează/Decelerează spre țintă
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
    Serial.println("Teensy UART Receiver Initialized. Waiting for data...");

    runPongLoop();
}