#include <Arduino.h>
#include "TeensyTimerTool.h"
#include "MT6835_encoder.h"
#include "wiring.h"

//NOTE: using ENCODER_PPR as both encoder's resolution AND driver's microstepping for ease of use
// maybe use smaller motor microstepping? - for finer control than sensor's accuracy?

#define MAX_ANGLE_ERROR 0.05
#define RPM_TO_US(rpm)  (60000000.0f / ((rpm) * ENCODER_PPR))
#define US_TO_RPM(us)   (60000000.0f / ((us) * ENCODER_PPR))

class stepperWrapper{
    uint8_t _dir, _step;
    TeensyTimerTool::PeriodicTimer _sTimer;
    MT6835* _encoder = NULL;
    volatile long long _steps = 0; // used only for running without encoder
    volatile bool _direction = HIGH;
    volatile long long _targetStep = 0;
    uint8_t _stepperID;
    bool _runWithoutEncoder = false;

public:
    stepperWrapper(uint8_t dir, uint8_t step, MT6835* encoder)
        : _dir(dir), _step(step), _encoder(encoder)
    {
        pinMode(_dir, OUTPUT);
        pinMode(_step, OUTPUT);
        if (_encoder) _encoder->begin();
    }

    void stepCallback();
    void stepCallbackNoEncoder();
    void initTimerFreq(float intervalUs);
    void setTimerFreq(float intervalUs);
    void setRunWithoutEncoder(bool val) {_runWithoutEncoder = val;}
    bool isAtTarget();
    void setTargetStep(long long target) {_targetStep = target;}
    MT6835* getEncoder() {return _encoder;}
};
