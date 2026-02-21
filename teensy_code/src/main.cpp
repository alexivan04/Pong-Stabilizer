#include "MT6835_encoder.h"
#include "core_pins.h"
#include "uartComm.h"
#include <cstdint>
#define USING_MAKEFILE
#include <Arduino.h>
#include <SPI.h>
#include "MT6835_encoder.h"
#include "stepperWrapper.h"
#include "uartComm.h"

//NOTE: lowered SPI comm frequency

//TODO:
// - write a getRevs function for autoCalibrate
// - test for the angleError
// - check for the ocasional SPI reading erros
// - try for the config to turn the motor a set amount of turns before looking for error
//      - consider the getRevs function here to count how many have been done
//      - could also use Z for revs? may be more accurate, look into it

#define CS_ENC           0
#define CAL_ENC          12
#define ENC_A            10
#define ENC_B            11
#define DEFAULT_RPM      400
#define CALIBRATION_FREQ 0x3 //matches 200 - 400 range from datasheet

MT6835 encoder_1(&SPI1, CS_ENC, CAL_ENC, ENC_A, ENC_B);
stepperWrapper stepper_1(3, 4, NULL);
stepperWrapper stepper_2(5, 6, NULL);
stepperWrapper stepper_3(7, 8, NULL);
stepperWrapper stepper_4(9, 10, NULL);

void encoder_isr_wrapper() {
    encoder_1.handleISR();
}

uint32_t getRevs() {
    return abs(stepper_1.getSteps()) / MOTOR_PPR;
}

void runContinousTest() {
    stepper_1.setRunWithoutEncoder(true); // must be called BEFORE init freq if no encoder used
    stepper_1.setRunWithoutPosition(true);
    stepper_1.initTimerFreqRPM(DEFAULT_RPM);

    stepper_2.setRunWithoutEncoder(true); // must be called BEFORE init freq if no encoder used
    stepper_2.setRunWithoutPosition(true);
    stepper_2.initTimerFreqRPM(DEFAULT_RPM);

    stepper_3.setRunWithoutEncoder(true); // must be called BEFORE init freq if no encoder used
    stepper_3.setRunWithoutPosition(true);
    stepper_3.initTimerFreqRPM(DEFAULT_RPM);

    stepper_4.setRunWithoutEncoder(true); // must be called BEFORE init freq if no encoder used
    stepper_4.setRunWithoutPosition(true);
    stepper_4.initTimerFreqRPM(DEFAULT_RPM);

    delay(1000);

    // stepper_1Encoder()->checkHealth();

    while(1) {
    //     Serial.printf("angle: %.4f | pulses: %ld\n", 
    //                   stepper_1.getEncoder()->getAngle(), 
    //                   stepper_1.getEncoder()->getRawPulses());
        // stepper_1.getEncoder()->checkHealth();
        delay(100);
    }
}

void runByHandTest() {
    delay(1000);

    stepper_1.getEncoder()->checkHealth();

    while(1) {
        Serial.printf("angle: %.4f | pulses: %ld\n", 
                      stepper_1.getEncoder()->getAngle(), 
                      stepper_1.getEncoder()->getRawPulses());
        stepper_1.getEncoder()->checkHealth();
        delay(100);
    }
}

void runCalibrationTest() {
    stepper_1.setDirection(HIGH);
    stepper_1.setRunWithoutEncoder(true); // must be called BEFORE init freq if no encoder used
    stepper_1.setRunWithoutPosition(true);
    stepper_1.initTimerFreqRPM(DEFAULT_RPM); // Pulse freq

    // stepper_1.setTargetStep(1000);
    stepper_1.getEncoder()->checkHealth();
    Serial.printf("zero pos: %f", stepper_1.getEncoder()->getZeroPos() * ZERO_POS_FINESSE);

    // let it run a bit to calibrate everything
    delay(500);
    // set proper calibration freq in EEPROM
    stepper_1.getEncoder()->setZeroPos();
    if(stepper_1.getEncoder()->getFrequencyRange() != CALIBRATION_FREQ) {
        stepper_1.getEncoder()->setFrequencyRange(CALIBRATION_FREQ);
        stepper_1.getEncoder()->programEEPROM();
        return;
    }
    else Serial.println("already configured at proper speed from EEPROM");

    if (stepper_1.getEncoder()->autoCalibrate(getRevs)) {
        Serial.println("system calibrated");
    }

    while(1) {
        Serial.printf("angle: %.4f | pulses: %ld\n", 
                      stepper_1.getEncoder()->getAngle(), 
                      stepper_1.getEncoder()->getRawPulses());
        stepper_1.getEncoder()->checkHealth();
        delay(100);
    }
}

void runStutterTest(int stutterTargetDegrees) {
    int dest = stutterTargetDegrees;
    stepper_1.initTimerFreqRPM(DEFAULT_RPM);
    stepper_1.setTargetStep(round(DEG_TO_STEPS(dest)));
    Serial.println(DEG_TO_STEPS(stutterTargetDegrees));
    delay(1000);

    stepper_1.getEncoder()->checkHealth();

    while(1) {
        if(stepper_1.isAtTarget()) {
            if (dest == 0) dest = stutterTargetDegrees;
            else dest = 0;
            stepper_1.setTargetStep(round(DEG_TO_STEPS(dest)));
            Serial.printf("REACHED angle: %.4f | pulses: %ld\n", 
                        stepper_1.getEncoder()->getAngle(), 
                        stepper_1.getEncoder()->getRawPulses());
        }
        stepper_1.getEncoder()->checkHealth();
        Serial.printf("angle: %.4f | pulses: %ld\n", 
                    stepper_1.getEncoder()->getAngle(), 
                    stepper_1.getEncoder()->getRawPulses());
        delay(10);
    }
}

void runSpeedRampTest() {
    const int minRPM = 50;
    const int maxRPM = 2000;
    const int stepRPM = 50; // increment per step
    const int delayMs = 500; // time to hold each speed

    stepper_1.setRunWithoutEncoder(true); // use encoder for feedback
    stepper_1.setDirection(true);

    while(true) {
        for(int rpm = minRPM; rpm <= maxRPM; rpm += stepRPM) {
            stepper_1.initTimerFreqRPM(rpm);
            Serial.printf("ramping to %d rpm\n", rpm);

            unsigned long start = millis();
            // while(millis() - start < delayMs) {
            //     Serial.printf("rpm: %d | angle: %.4f | pulses: %ld\n",
            //                   rpm,
            //                   stepper_1.getEncoder()->getAngle(),
            //                   stepper_1.getEncoder()->getRawPulses());
            //     stepper_1.getEncoder()->checkHealth();
            //     delay(50);
            // }
            delay(delayMs);
        }
    }
    Serial.println("speed ramp test completed");
}

int main() {
    Serial.begin(115200);
    Serial1.begin(115200);
    delay(250);
    while (!Serial);
    Serial.println("Teensy UART Receiver Initialized. Waiting for data...");

    // ABZ rate was not configured in EEPROM, write and exit
    // if(stepper_1.getEncoder()->begin() == -1) return 1;

    // this has to be called here, function has to be static (cant do it in class)
    // attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_isr_wrapper, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(ENC_B), encoder_isr_wrapper, CHANGE);

    // stepper_1.setRunWithoutEncoder(true); // must be called BEFORE init freq if no encoder used
    // runCalibrationTest();
    // runStutterTest(180);
    // runCalibrationTest();
    // runSpeedRampTest();
    // Serial.printf("zero pos: %f", stepper_1.getEncoder()->getZeroPos() * ZERO_POS_FINESSE);
    // delay(2000);
    // stepper_1.getEncoder()->setZeroPos();
    // Serial.printf("zero pos: %f", stepper_1.getEncoder()->getZeroPos() * ZERO_POS_FINESSE);
    while (1) {
        checkSerial();
    }
}
