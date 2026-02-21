#include <Arduino.h>
#include "TeensyTimerTool.h"
#include "MT6835_encoder.h"
#include "wiring.h"
#include <exception>

#define MAX_ANGLE_ERROR 0.075
#define RPM_TO_US(rpm)  (60000000.0f / ((rpm) * ENCODER_PPR))
#define US_TO_RPM(us)   (60000000.0f / ((us) * ENCODER_PPR))

// this is used ONLY if NO encoder is used
#define MOTOR_PPR                 6400
#define RPM_TO_US_NO_ENCODER(rpm) (60000000.0f / ((rpm) * MOTOR_PPR))
#define US_TO_RPM_NO_ENCODER(us)  (60000000.0f / ((us) * MOTOR_PPR))

//NOTE: By default, motor uses data from ENCODER to determine if tartget has been reached
//      runWithoutEncoder == true - will determine if target step has been reached from MOTOR DRIVER ticks
//      runWithoutPosition == true - will NOT use any check for a target step, only direction

class stepperWrapper{
public:
    stepperWrapper(uint8_t dir, uint8_t step, MT6835* encoder)
        : _dir(dir), _step(step), _encoder(encoder)
    {
        pinMode(_dir, OUTPUT);
        pinMode(_step, OUTPUT);
    }

    void stepCallback();
    void stepCallbackNoEncoder();
    void initTimerFreqUS(float intervalUs);
    void setTimerFreqUS(float intervalUs);
    void initTimerFreqRPM(float intervalRPM);
    void setTimerFreqRPM(float intervalRPM);
    void setRunWithoutEncoder(bool val) {_runWithoutEncoder = val;}
    void setRunWithoutPosition(bool val) {_runWithoutPosition = val;}
    void setDirection(bool val) {_direction = val;}
    bool getDirection() {return _direction;}
    bool isAtTarget();
    void setTargetStep(long long target) {_targetStep = target;}
    MT6835* getEncoder() {return _encoder;}
    long long getSteps() {return _steps;}

private:
    uint8_t _dir, _step;
    TeensyTimerTool::PeriodicTimer _sTimer;
    MT6835* _encoder = NULL;
    volatile long long _steps = 0; // used only for running without encoder
    volatile bool _direction = HIGH;
    volatile long long _targetStep = 0;
    uint8_t _stepperID;
    bool _runWithoutEncoder = false;
    bool _runWithoutPosition = false; // for continuous movement in a direction
};
