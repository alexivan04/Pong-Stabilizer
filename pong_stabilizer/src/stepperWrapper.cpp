#include "stepperWrapper.h"
#include "MT6835_encoder.h"

void stepperWrapper::stepCallback() {
    if (isAtTarget()) return;

    digitalToggleFast(_step);

    // not incrementing _steps to avoid improper incrementing
    // in case of mecanically skipped steps
    // tracking is done by pulses reported by encoder
    if (!_runWithoutPosition) {
        if (_encoder->getRawPulses() < _targetStep) {
            _direction = HIGH;
            // _steps++;
        } else {
            _direction = LOW;
            // _steps--;
        }
    }
    digitalWriteFast(_dir, _direction);
}

void stepperWrapper::stepCallbackNoEncoder() {
    if(_steps == _targetStep) return;

    digitalToggleFast(_step);

    if (!_runWithoutPosition) {
        if (_steps < _targetStep) {
            _direction = HIGH;
            _steps++;
        } else {
            _direction = LOW;
            _steps--;
        }
    }
    digitalWriteFast(_dir, _direction);
}

void stepperWrapper::initTimerFreqUS(float intervalUs) {
    if(_encoder == NULL || _runWithoutEncoder)
        _sTimer.begin([this] {stepCallbackNoEncoder();}, intervalUs);
    else _sTimer.begin([this] {stepCallback();}, intervalUs);
}

void stepperWrapper::setTimerFreqUS(float intervalUs) {
    _sTimer.setPeriod(intervalUs);
}

void stepperWrapper::initTimerFreqRPM(float intervalRPM) {
    if(_encoder == NULL || _runWithoutEncoder)
        _sTimer.begin([this] {stepCallbackNoEncoder();}, RPM_TO_US_NO_ENCODER(intervalRPM));
    else _sTimer.begin([this] {stepCallback();}, RPM_TO_US(intervalRPM));
}

void stepperWrapper::setTimerFreqRPM(float intervalRPM) {
    if(_encoder == NULL || _runWithoutEncoder)
        _sTimer.setPeriod(RPM_TO_US_NO_ENCODER(intervalRPM));
    else _sTimer.setPeriod(RPM_TO_US(intervalRPM));
}

bool stepperWrapper::isAtTarget() {
    return (abs(_encoder->getRawPulses() - _targetStep) < DEG_TO_STEPS(MAX_ANGLE_ERROR));

}
