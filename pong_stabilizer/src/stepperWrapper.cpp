#include "stepperWrapper.h"
#include "MT6835_encoder.h"


void stepperWrapper::stepCallback() {
    if (isAtTarget()) return;

    digitalToggleFast(_step);

    // not incrementing _steps to avoid improper incrementing
    // in case of mecanically skipped steps
    // tracking is done by pulses reported by encoder
    if (_encoder->getRawPulses() < _targetStep) {
        _direction = HIGH;
        // _steps++;
    } else {
        _direction = LOW;
        // _steps--;
    }
    digitalWriteFast(_dir, _direction);
}

void stepperWrapper::stepCallbackNoEncoder() {
    if(_steps == _targetStep) return;

    digitalToggleFast(_step);

    if (_steps < _targetStep) {
        _direction = HIGH;
        _steps++;
    } else {
        _direction = LOW;
        _steps--;
    }
    digitalWriteFast(_dir, _direction);
}

void stepperWrapper::initTimerFreq(float intervalUs) {
    if(_encoder == NULL || _runWithoutEncoder)
        _sTimer.begin([this] {stepCallbackNoEncoder();}, intervalUs);
    else _sTimer.begin([this] {stepCallback();}, intervalUs);
}

void stepperWrapper::setTimerFreq(float intervalUs) {
    _sTimer.setPeriod(intervalUs);
}

bool stepperWrapper::isAtTarget() {
    return (abs(_encoder->getRawPulses() - _targetStep) < DEG_TO_STEPS(MAX_ANGLE_ERROR));

}
