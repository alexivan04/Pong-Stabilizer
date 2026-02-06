#include "MT6835_encoder.h"
#include "core_pins.h"
#include <cstdint>
#define USING_MAKEFILE
#include <Arduino.h>
#include <SPI.h>
#include "MT6835_encoder.h"
#include "stepperWrapper.h"

//TODO: 
// - write a getRevs function for autoCalibrate
// - test for the angleError
// - test for value of motor stepping when using encoder (CURRENLY BOTH PPR ARE EQUAL)
// - check for the ocasional SPI reading erros
// - try for the config to turn the motor a set amount of turns before looking for error
//      - consider the getRevs function here to count how many have been done
//      - could also use Z for revs? may be more accurate, look into it


// Define Pins
#define CS_ENC 0
#define CAL_ENC 12
#define ENC_A 10
#define ENC_B 11
#define DEFAULT_RPM 200

// Instances
MT6835 encoder_1(&SPI1, CS_ENC, CAL_ENC, ENC_A, ENC_B);
stepperWrapper stepper_1(3, 4, &encoder_1);

// --- ISR Bridge ---
// Since the ISR must be a static or global function, we wrap the class method:
void encoder_isr_wrapper() {
    encoder_1.handleISR();
}

void runContinousTest() {
    stepper_1.setRunWithoutEncoder(true); // must be called BEFORE init if no encoder used
    stepper_1.initTimerFreq(RPM_TO_US(DEFAULT_RPM));
    delay(1000);

    stepper_1.getEncoder()->checkHealth();

    if(stepper_1.getEncoder()->getFrequencyRange() != 0b0101) {
        stepper_1.getEncoder()->setFrequencyRange(0b101);
        stepper_1.getEncoder()->programEEPROM();
        return;
    }
    else Serial.println("already configured at proper speed from EEPROM");

    while(1) {
        Serial.printf("Angle: %.4f | Pulses: %ld\n", 
                      stepper_1.getEncoder()->getAngle(), 
                      stepper_1.getEncoder()->getRawPulses());
        stepper_1.getEncoder()->checkHealth();
        delay(100);
    }
}

void runByHandTest() {
    delay(1000);

    stepper_1.getEncoder()->checkHealth();

    if(stepper_1.getEncoder()->getFrequencyRange() != 0b0101) {
        // delay(500);
        stepper_1.getEncoder()->setFrequencyRange(0b101);
        stepper_1.getEncoder()->programEEPROM();
        return;
    }
    else Serial.println("already configured at proper speed from EEPROM");

    while(1) {
        Serial.printf("Angle: %.4f | Pulses: %ld\n", 
                      stepper_1.getEncoder()->getAngle(), 
                      stepper_1.getEncoder()->getRawPulses());
        stepper_1.getEncoder()->checkHealth();
        delay(100);
    }
}

void runCalibrationTest() {
    stepper_1.setRunWithoutEncoder(true);
    stepper_1.initTimerFreq(RPM_TO_US(DEFAULT_RPM)); // Pulse freq
    delay(1000);

    // stepper_1.setTargetStep(1000);
    stepper_1.getEncoder()->checkHealth();

    if(stepper_1.getEncoder()->getFrequencyRange() != 0b101) {
        // delay(500);
        stepper_1.getEncoder()->setFrequencyRange(0b101);
        stepper_1.getEncoder()->programEEPROM();
        return;
    }
    else Serial.println("already configured at proper speed from EEPROM");

    if (stepper_1.getEncoder()->autoCalibrate()) {
        Serial.println("System Calibrated.");
    }

    while(1) {
        Serial.printf("Angle: %.4f | Pulses: %ld\n", 
                      stepper_1.getEncoder()->getAngle(), 
                      stepper_1.getEncoder()->getRawPulses());
        stepper_1.getEncoder()->checkHealth();
        delay(100);
    }
}

void runStutterTest() {
    stepper_1.initTimerFreq(RPM_TO_US(DEFAULT_RPM)); // Pulse freq
    int dest = 90;
    stepper_1.setTargetStep(round(DEG_TO_STEPS(dest)));
    delay(1000);

    stepper_1.getEncoder()->checkHealth();

    uint8_t range = stepper_1.getEncoder()->getFrequencyRange();
    Serial.println(range);
    if(range != 0b011) {
        delay(10);
        uint8_t new_range = stepper_1.getEncoder()->getFrequencyRange();
        Serial.println(new_range);
        stepper_1.getEncoder()->setFrequencyRange(0b011);
        stepper_1.getEncoder()->programEEPROM();
        return;
    }
    else Serial.println("already configured at proper speed from EEPROM");

    while(1) {
        if(stepper_1.isAtTarget()) {
            if (dest == 0) dest = 90;
            else dest = 0;
            stepper_1.setTargetStep(round(DEG_TO_STEPS(dest)));
            Serial.printf("REACHED Angle: %.4f | Pulses: %ld\n", 
                        stepper_1.getEncoder()->getAngle(), 
                        stepper_1.getEncoder()->getRawPulses());
        }
        stepper_1.getEncoder()->checkHealth();
        delay(10);
    }
}

int main() {
    Serial.begin(115200);

    // this has to be called here, function has to be static (cant do it in class)
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_isr_wrapper, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoder_isr_wrapper, CHANGE);
    delay(5000);

    runStutterTest();
    // runByHandTest();
}
