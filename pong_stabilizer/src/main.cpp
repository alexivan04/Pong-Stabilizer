#define USING_MAKEFILE
#include <Arduino.h>
#include <SPI.h>
#include "stepperWrapper.h"

// --- Configuration ---
#define SERVO_TIMER_US 200
#define STEPPER_PULSE_FREQ 0.04ms

// configured as 1/16 microstepping
#define MICROSTEPS_PER_REV  (200*16)  // 3200
#define REVS                240
#define TOTAL_STEPS         (MICROSTEPS_PER_REV * REVS)  // 204800

// --- Encoder Resolution ---
#define PPR 16384.0f
#define TOTAL_COUNTS_PER_REV (PPR * 4.0f)

// Pins
#define MS1   5
#define MS2   6
#define MS3   7
#define DIR_1   3
#define STEP_1  4
#define ENC_A_1 10
#define ENC_B_1 11
// Using SPI1 with mode 3
// MOSI - 26
// MISO - 1
// SCK - 27
#define CS 0
#define CONFIG_ENCODER 12
// #undef PIN_SPI_SS
// #undef PIN_SPI_MOSI
// #undef PIN_SPI_MISO
// #undef PIN_SPI_SCK

// #define PIN_SPI_SS    0
// #define PIN_SPI_MOSI  26
// #define PIN_SPI_MISO  1
// #define PIN_SPI_SCK   27

// --- Globals ---
volatile int32_t raw_pulses = 0;
volatile uint8_t prev_state = 0;

// Filtered Variables
float current_angle = 0.0f;     // Exact Angle in Degrees (0-360 or cumulative)
float filtered_velocity = 0.0f; // Smoothed Degrees per Second
float last_angle = 0.0f;

// Filter Tuning
float alpha = 0.2f; // Lower = smoother/less wobble, higher = more responsive

const int8_t LOOKUP[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Testing stepper
stepperWrapper stepper_1 = stepperWrapper(DIR_1, STEP_1, MS1, MS2, MS3);

void encoder_isr() {
    uint8_t state = (digitalReadFast(ENC_A_1) << 1) | digitalReadFast(ENC_B_1);
    raw_pulses += LOOKUP[(prev_state << 2) | state];
    prev_state = state;
}

int main() {
    Serial.begin(115200);
    pinMode(ENC_A_1, INPUT_PULLUP);
    pinMode(ENC_B_1, INPUT_PULLUP);
    pinMode(CONFIG_ENCODER, OUTPUT);
    prev_state = (digitalReadFast(ENC_A_1) << 1) | digitalReadFast(ENC_B_1);
    attachInterrupt(digitalPinToInterrupt(ENC_A_1), encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_1), encoder_isr, CHANGE);

    SPI1.begin();
    pinMode(CS, OUTPUT);
    digitalWriteFast(CS, HIGH);

    delay(5000);


    // read register 0x00E - bits 6-4 contain autocal_freq
    SPI1.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS, LOW);
    delayNanoseconds(100);   // meet tl
    SPI1.transfer16(0b0011 << 12 | 0x00E);
    uint8_t recva = SPI1.transfer(0); //got the register 0x00E
    digitalWrite(CS, HIGH);

    uint8_t old_autocal = (recva >> 4) & 0x07;
    printf("old AUTOCAL_FREQ = %d\n", old_autocal);  // prints 3-bit binary

    // modify ONLY the bits 6-4
    uint8_t autocal = 0b011;
    recva = (recva & 0x8F) | ((autocal & 0x07) << 4);  

    // write in memory
    digitalWrite(CS, LOW);
    delayNanoseconds(100);
    SPI1.transfer16(0b0110 << 12 | 0x00E); // write command, depends on your chip
    SPI1.transfer(recva);
    digitalWrite(CS, HIGH);

    // read register 0x00E - bits 6-4 contain autocal_freq
    recva = 0;
    // SPI1.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS, LOW);
    delayNanoseconds(100);   // meet tl
    SPI1.transfer16(0b0011 << 12 | 0x00E);
    recva = SPI1.transfer(0); //got the register 0x00E
    digitalWrite(CS, HIGH);
    uint8_t new_autocal = (recva >> 4) & 0x07;
    printf("new AUTOCAL_FREQ = %d\n", new_autocal);

    delay(2000);

    // prepare for auto calibration
    stepper_1.setMicrosteps(LOW, LOW, HIGH);
    stepper_1.initTimerFreq(STEPPER_PULSE_FREQ);
    stepper_1.setDir(false);
    stepper_1.setRunWithoutTarget(true);
    
    
    // stepper_1.setStep(TOTAL_STEPS);
    digitalWriteFast(CONFIG_ENCODER, LOW);
    delay(2000); // optional settling
    digitalWriteFast(CONFIG_ENCODER, HIGH); // pull CAL_EN high
    delay(100); // optional settling
    

    uint32_t last_rev = 0;
    bool calibration_done = false;
    uint8_t cal_status;
    while(!calibration_done) {
        // stepper spinning in background via timer
        uint32_t revs_done = stepper_1.steps / MICROSTEPS_PER_REV;
        if(revs_done > last_rev) {
            last_rev = revs_done;
            Serial.print("Revolution done: ");
            Serial.println(last_rev);
        }

        // read calibration status from 0x113[7:6]
        // SPI1.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3));
        digitalWrite(CS, LOW);
        delayNanoseconds(100);
        SPI1.transfer16(0b0011 << 12 | 0x113);
        uint8_t reg113 = SPI1.transfer(0);
        digitalWrite(CS, HIGH);
        SPI1.endTransaction();

        cal_status = (reg113 >> 6) & 0x03; // bits 7-6
        switch(cal_status) {
            case 0b00: Serial.println("No calibration"); break;
            case 0b01: Serial.println("Calibration running"); break;
            case 0b10: 
                Serial.println("Calibration FAILED"); 
                calibration_done = true; 
                break;
            case 0b11: 
                Serial.println("Calibration SUCCESS"); 
                calibration_done = true; 
                break;
        }

        delay(10); // short delay to avoid spamming SPI / Serial
    }

    // optionally wait >6s after success
    if(cal_status == 0b11) delay(6000);

    // power down or continue with normal operation
    digitalWriteFast(CONFIG_ENCODER, LOW);
    Serial.println("calibration process finished");

    

    // for (int i = 0; i < 5; i++) {
    //     // uint32_t now = micros();

    //     // // Fixed Frequency Loop (0.75ms)
    //     // if (now - last_loop >= SERVO_TIMER_US) {
    //     //     float dt = (now - last_loop) / 1000000.0f; // Actual time delta in seconds
    //     //     last_loop = now;

    //     //     // 1. CALCULATE EXACT ANGLE
    //     //     // Cumulative angle (e.g., can go to 720, 1080... useful for balancing)
    //     //     current_angle = ((float)raw_pulses * 360.0f) / TOTAL_COUNTS_PER_REV;

    //     //     // 2. CALCULATE VELOCITY (Degrees per Second)
    //     //     float raw_velocity = (current_angle - last_angle) / dt;

    //     //     // 3. APPLY FILTER TO VELOCITY
    //     //     // This removes the "noise" from the off-center magnet/wobble
    //     //     filtered_velocity = (alpha * raw_velocity) + ((1.0f - alpha) * filtered_velocity);

    //     //     last_angle = current_angle;
    //     // }

    //     // if(millis() - last_dir_switch_time > delay_swithc_dir) {
    //     //     dest = -dest;
    //     //     stepper_1.setStep(dest);
    //     //     last_dir_switch_time = millis();
    //     // }

    //     // // // Print data every 50ms for the Serial Plotter
    //     // if (millis() - last_print > 50) {
    //     //     last_print = millis();
    //     //     // Serial.print("Angle:"); 
    //     //     Serial.println(current_angle, 4); // 4 decimal places for "Exact" precision
    //     //     // Serial.print(",");
    //     //     // Serial.print("Vel:"); 
    //     //     // Serial.println(filtered_velocity, 2);
    //     // }
    //     // Serial.println(raw_pulses, 4); // 4 decimal places for "Exact" precision
    //     delay(1000);
    // }

}
