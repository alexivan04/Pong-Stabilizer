#define USING_MAKEFILE
#include <Arduino.h>
#include "stepperWrapper.h"

// TODO: 
//      - UART communication with PI (packet structure unknown yet)
//      - determine how movements work for balancing and bouncing a ball
//      - how to corelate ms frequency to stepper speed and acceleration

#define SERVO_TIMER_FREQ 0.1ms

// common for all steppers
#define MS1   5
#define MS2  6
#define MS3   7

// individual settings
#define DIR_1   3
#define STEP_1  4
#define ENC_A_1 10
#define ENC_B_1 11

#define DIR_2   3
#define STEP_2  4
#define ENC_A_2 10
#define ENC_B_2 11

#define DIR_3   3
#define STEP_3  4
#define ENC_A_2 10
#define ENC_B_2 11

#define DIR_4   3
#define STEP_4  4
#define ENC_A_2 10
#define ENC_B_2 11

stepperWrapper stepper_1 = stepperWrapper(DIR_1, STEP_1, MS1, MS2, MS3);
stepperWrapper stepper_2 = stepperWrapper(DIR_2, STEP_2, MS1, MS2, MS3);
stepperWrapper stepper_3 = stepperWrapper(DIR_3, STEP_3, MS1, MS2, MS3);
stepperWrapper stepper_4 = stepperWrapper(DIR_4, STEP_4, MS1, MS2, MS3);

int main() {
    // 1/16 microstepping
    stepper_1.setMicrosteps(LOW, LOW, HIGH);
    stepper_2.setMicrosteps(LOW, LOW, HIGH);
    stepper_3.setMicrosteps(LOW, LOW, HIGH);
    stepper_4.setMicrosteps(LOW, LOW, HIGH);

    stepper_1.initTimerFreq(SERVO_TIMER_FREQ);
    stepper_2.initTimerFreq(SERVO_TIMER_FREQ);
    stepper_3.initTimerFreq(SERVO_TIMER_FREQ);
    stepper_4.initTimerFreq(SERVO_TIMER_FREQ);
    
    return 0;
}
