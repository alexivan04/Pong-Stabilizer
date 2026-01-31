#include <Arduino.h>
#include "TeensyTimerTool.h"

struct stepperWrapper{
    uint8_t DIR_pin, STEP_pin, MS1_pin, MS2_pin, MS3_pin;
    TeensyTimerTool::PeriodicTimer sTimer;
    volatile long long steps = 0;
    volatile bool dir = HIGH;
    volatile long targetStep = 0;
    uint8_t stepperID;
    bool run_without_pos = false;

    stepperWrapper(uint8_t DIR_pin, uint8_t STEP_pin, uint8_t MS1_pin, uint8_t MS2_pin, uint8_t MS3_pin) {
        this->DIR_pin = DIR_pin;
        this->STEP_pin = STEP_pin;
        this->MS1_pin = MS1_pin;
        this->MS2_pin = MS2_pin;
        this->MS3_pin = MS3_pin;

        pinMode(this->DIR_pin, OUTPUT);
        pinMode(this->STEP_pin, OUTPUT);
        pinMode(this->MS1_pin, OUTPUT);
        pinMode(this->MS2_pin, OUTPUT);
        pinMode(this->MS3_pin, OUTPUT);

        // no microstepping
        setMicrosteps(LOW, LOW, LOW);
    }

    void stepCallback();
    void setMicrosteps(uint8_t MS1_val, uint8_t MS2_val, uint8_t MS3_val);
    void initTimerFreq(duration<double,std::milli> intervalMs);
    void setTimerFreq(duration<double,std::milli> intervalMs);
    void setStep(long target) {this->targetStep = target;}
    bool isAtTarget() {return (this->targetStep == this->steps);}
    void setRunWithoutTarget(bool val) {this->run_without_pos = val;}
    void setDir(bool val) {this->dir = val;}
};
