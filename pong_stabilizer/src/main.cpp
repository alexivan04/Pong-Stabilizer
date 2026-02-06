#include "MT6835_encoder.h"
#include "core_pins.h"
#include <cstdint>
#define USING_MAKEFILE
#include <Arduino.h>
#include <SPI.h>
#include "MT6835_encoder.h"
#include "stepperWrapper.h"

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
#define DEFAULT_RPM      200
#define CALIBRATION_FREQ 0x4 //matches 200 - 400 range from datasheet

MT6835 encoder_1(&SPI1, CS_ENC, CAL_ENC, ENC_A, ENC_B);
stepperWrapper stepper_1(3, 4, &encoder_1);

void encoder_isr_wrapper() {
    encoder_1.handleISR();
}

void runContinousTest() {
    // stepper_1.setRunWithoutEncoder(true); // must be called BEFORE init freq if no encoder used
    stepper_1.setRunWithoutPosition(true);
    stepper_1.initTimerFreqRPM(DEFAULT_RPM);
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
    // stepper_1.setRunWithoutEncoder(true); // must be called BEFORE init freq if no encoder used
    stepper_1.setRunWithoutPosition(true);
    stepper_1.initTimerFreqRPM(DEFAULT_RPM); // Pulse freq
    delay(1000);

    // stepper_1.setTargetStep(1000);
    stepper_1.getEncoder()->checkHealth();

    // set proper calibration freq in EEPROM
    if(stepper_1.getEncoder()->getFrequencyRange() != CALIBRATION_FREQ) {
        delay(100);
        stepper_1.getEncoder()->setFrequencyRange(CALIBRATION_FREQ);
        stepper_1.getEncoder()->programEEPROM();
        return;
    }
    else Serial.println("already configured at proper speed from EEPROM");

    if (stepper_1.getEncoder()->autoCalibrate()) {
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
        delay(10);
    }
}

int main() {
    Serial.begin(115200);

    // ABZ rate was not configured in EEPROM, write and exit
    if(stepper_1.getEncoder()->begin() == -1) return 1;

    // this has to be called here, function has to be static (cant do it in class)
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_isr_wrapper, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoder_isr_wrapper, CHANGE);
    delay(5000);

    // stepper_1.setRunWithoutEncoder(true); // must be called BEFORE init freq if no encoder used
    runStutterTest(20);
}
