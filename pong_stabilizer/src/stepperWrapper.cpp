#include "stepperWrapper.h"


void stepperWrapper::stepCallback() {
    // step pulse
    if((this->steps == this->targetStep) && !this->run_without_pos) return;

    digitalToggleFast(this->STEP_pin);

    if (run_without_pos) {
        if (this->dir == HIGH)
            this->steps++;
        else this->steps--;
        digitalWriteFast(this->DIR_pin, this->dir);
    }

    else {
        // update direction towards target
        if (this->steps < targetStep) {
            // if (this->dir == 0) {
                this->dir = HIGH;
                digitalWriteFast(this->DIR_pin, HIGH);
            // }
            this->steps++;
        } else if (this->steps > targetStep) {
            // if (this->dir == 1) {
                this->dir = LOW;
                digitalWriteFast(this->DIR_pin, LOW);
            // }
            this->steps--;
        }
    }
}

void stepperWrapper::setMicrosteps(uint8_t MS1_val, uint8_t MS2_val, uint8_t MS3_val) {
    digitalWriteFast(this->MS1_pin, MS1_val);
    digitalWriteFast(this->MS2_pin, MS2_val);
    digitalWriteFast(this->MS3_pin, MS3_val);
}

void stepperWrapper::initTimerFreq(duration<double,std::milli> intervalMs) {
    sTimer.begin([this] {stepCallback();}, intervalMs);
}

void stepperWrapper::setTimerFreq(duration<double,std::milli> intervalMs) {
    sTimer.setPeriod(intervalMs);
}
