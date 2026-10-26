#ifndef STEPPER_WRAPPER_H
#define STEPPER_WRAPPER_H

#include <Arduino.h>
#include "TeensyTimerTool.h"
#include "MT6835_encoder.h"

// mechanical settings
#define MOTOR_PPR 3200.0f
#define DEG_TO_MOTOR_STEPS(deg) ((deg) * MOTOR_PPR / 360.0f)

class stepperWrapper{
public:
    stepperWrapper(uint8_t dir, uint8_t step, MT6835* encoder);

    void begin();
    void update();

    void setTargetStep(long long target) { _targetStep = target; }
    void setCurrentPositionInSteps(long long steps);
    long long getSteps();
    // one step pulse straight to the driver, not counted in the position
    // (the start-up tune pairs every pulse with one back, so it nets to zero)
    void rawPulse(bool positive);

    void setTargetRPM(float rpm); 
    void setRunWithoutPosition(bool state) { _runWithoutPosition = state; }

    void setMaxRPM(float rpm);
    void setAccelerationRate(float accelRate);

private:
    void tick();
    long long getActualPosition();

    uint8_t _dir, _step;
    TeensyTimerTool::PeriodicTimer _sTimer;
    MT6835* _encoder;

    bool _runWithoutPosition = false;

    volatile long long _internalSteps = 0;
    volatile long long _targetStep = 0;

    volatile float _currentSpeed = 0;
    float _targetSpeed = 0;

    float _maxSpeed = 10000.0f; // ~1100 deg/s; balancing needs < 1000 steps/s, and the
                                // old 35000 lost steps in violent moves
    float _accel = 90.0f;      // accel rate per update
    float _Kp = 25.0f;          // aggressiveness

    volatile float _accumulator = 0;
    volatile bool _stepPulseActive = false;
};

#endif
