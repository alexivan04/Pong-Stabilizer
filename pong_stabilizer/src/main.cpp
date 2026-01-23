#define USING_MAKEFILE
#include <Arduino.h>
#include "TeensyTimerTool.h"
#include "stepperWrapper.h"

#define SERVO_TIMER_FREQ 0.1ms

// common for all steppers
#define MS1   5
#define MS2  6
#define MS3   7

// individual settings
#define DIR_A   3
#define STEP_A  4

#define DIR_B   3
#define STEP_B  4

#define DIR_C   3
#define STEP_C  4

stepperWrapper stepper_A = stepperWrapper(DIR_A, STEP_A, MS1, MS2, MS3);
stepperWrapper stepper_B = stepperWrapper(DIR_B, STEP_B, MS1, MS2, MS3);
stepperWrapper stepper_C = stepperWrapper(DIR_C, STEP_C, MS1, MS2, MS3);

int main() {
    stepper_A.setMicrosteps(LOW, LOW, HIGH);
    stepper_B.setMicrosteps(LOW, LOW, HIGH);
    stepper_C.setMicrosteps(LOW, LOW, HIGH);

    stepper_A.initTimerFreq(SERVO_TIMER_FREQ);
    stepper_B.initTimerFreq(SERVO_TIMER_FREQ);
    stepper_C.initTimerFreq(SERVO_TIMER_FREQ);
    
    return 0;
}
