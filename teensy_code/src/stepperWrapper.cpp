#include "stepperWrapper.h"

stepperWrapper::stepperWrapper(uint8_t dir, uint8_t step, MT6835* encoder)
    : _dir(dir), _step(step), _encoder(encoder) {}

void stepperWrapper::begin() {
    pinMode(_dir, OUTPUT);
    pinMode(_step, OUTPUT);
    
    _sTimer.begin([this] { this->tick(); }, 20);
}

void stepperWrapper::tick() {
    if (_stepPulseActive) {
        digitalWriteFast(_step, LOW);
        _stepPulseActive = false;
    }

    _accumulator += abs(_currentSpeed);
    
    if (_accumulator >= 50000.0f) {
        _accumulator -= 50000.0f;
        
        if (_currentSpeed > 0) {
            digitalWriteFast(_dir, HIGH);
            _internalSteps++;
        } else if (_currentSpeed < 0) {
            digitalWriteFast(_dir, LOW);
            _internalSteps--;
        }
        
        delayNanoseconds(250);
        
        digitalWriteFast(_step, HIGH);
        _stepPulseActive = true;
    }
}

// ==========================================
long long stepperWrapper::getActualPosition() {
    if (_encoder != nullptr) {
        long long rawPulses = _encoder->getRawPulses();
        return (rawPulses * (long long)MOTOR_PPR) / (long long)ENCODER_PPR;
    }
    // internal counter if no encoder
    return _internalSteps;
}

long long stepperWrapper::getSteps() {
    return getActualPosition();
}

void stepperWrapper::setCurrentPositionInSteps(long long steps) {
    _internalSteps = steps;
    _targetStep = steps;
    // offset pe encoder daca folosim
}

void stepperWrapper::setTargetRPM(float rpm) {
    _targetSpeed = (rpm / 60.0f) * MOTOR_PPR;
}

void stepperWrapper::setMaxRPM(float rpm) {
    _maxSpeed = (rpm / 60.0f) * MOTOR_PPR;
}

void stepperWrapper::setAccelerationRate(float accelRate) {
    _accel = accelRate; 
}

void stepperWrapper::update() {
    if (_runWithoutPosition) {
        // speed mode, no encoder
        if (_currentSpeed < _targetSpeed) {
            _currentSpeed += _accel;
            if (_currentSpeed > _targetSpeed) _currentSpeed = _targetSpeed;
        } else if (_currentSpeed > _targetSpeed) {
            _currentSpeed -= _accel;
            if (_currentSpeed < _targetSpeed) _currentSpeed = _targetSpeed;
        }
        
    } else {
        // with encoder
        long long currentPos = getActualPosition();
        long long error = _targetStep - currentPos;
        
        if (abs(error) <= 2) {
            _currentSpeed = 0;
            return;
        }

        float desiredSpeed = (float)error * _Kp;
        
        if (desiredSpeed > _maxSpeed) desiredSpeed = _maxSpeed;
        if (desiredSpeed < -_maxSpeed) desiredSpeed = -_maxSpeed;
        
        if (_currentSpeed < desiredSpeed) {
            _currentSpeed += _accel;
            if (_currentSpeed > desiredSpeed) _currentSpeed = desiredSpeed;
        } else if (_currentSpeed > desiredSpeed) {
            _currentSpeed -= _accel;
            if (_currentSpeed < desiredSpeed) _currentSpeed = desiredSpeed;
        }
    }
}
