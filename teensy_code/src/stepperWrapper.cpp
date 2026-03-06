#include "stepperWrapper.h"

stepperWrapper::stepperWrapper(uint8_t dir, uint8_t step, MT6835* encoder)
    : _dir(dir), _step(step), _encoder(encoder) {}

void stepperWrapper::begin() {
    pinMode(_dir, OUTPUT);
    pinMode(_step, OUTPUT);
    
    // Timer hardware non-stop la 50kHz (20 us)
    _sTimer.begin([this] { this->tick(); }, 20);
}

// ==========================================
// 1. HARDWARE TICK (NU SCHIMBĂ FRECVENȚA NICIODATĂ)
// ==========================================
void stepperWrapper::tick() {
    if (_stepPulseActive) {
        digitalWriteFast(_step, LOW);
        _stepPulseActive = false;
    }

    _accumulator += abs(_currentSpeed);
    
    if (_accumulator >= 50000.0f) {
        _accumulator -= 50000.0f;
        
        // Direcția logică
        if (_currentSpeed > 0) {
            digitalWriteFast(_dir, HIGH);
            _internalSteps++;
        } else if (_currentSpeed < 0) {
            digitalWriteFast(_dir, LOW);
            _internalSteps--;
        }
        
        delayNanoseconds(250); // Setup time obligatoriu
        
        digitalWriteFast(_step, HIGH);
        _stepPulseActive = true;
    }
}

// ==========================================
// 2. CITIRE ENCODER ȘI SCALARE
// ==========================================
long long stepperWrapper::getActualPosition() {
    if (_encoder != nullptr) {
        // Transformăm pulsurile brute ale encoderului în pași echivalenți pentru motor.
        // Asta rezolvă diferențele dacă encoderul e mai fin decât motorul.
        long long rawPulses = _encoder->getRawPulses();
        return (rawPulses * (long long)MOTOR_PPR) / (long long)ENCODER_PPR;
    }
    return _internalSteps; // Daca n-ai encoder, folosește numărătoarea internă
}

long long stepperWrapper::getSteps() {
    return getActualPosition();
}

void stepperWrapper::setCurrentPositionInSteps(long long steps) {
    _internalSteps = steps;
    _targetStep = steps;
    // Notă: Dacă folosești encoder, de obicei trebuie să faci offset și pe encoder aici.
}

// ==========================================
// 3. SETĂRI RPM ȘI ACCELERAȚIE PENTRU USER
// ==========================================
void stepperWrapper::setTargetRPM(float rpm) {
    // Convertim RPM în Pași per Secundă
    _targetSpeed = (rpm / 60.0f) * MOTOR_PPR;
}

void stepperWrapper::setMaxRPM(float rpm) {
    _maxSpeed = (rpm / 60.0f) * MOTOR_PPR;
}

void stepperWrapper::setAccelerationRate(float accelRate) {
    // Aici tu îi dădeai un accelRate. Îl mapăm la noul sistem
    _accel = accelRate; 
}

// ==========================================
// 4. CREIERUL MOTOARELOR (ACCELERARE + PID)
// ==========================================
void stepperWrapper::update() {
    if (_runWithoutPosition) {
        // --- MODUL VITEZĂ (Fără encoder, doar învârte) ---
        // Accelerează lin până la viteza dorită
        if (_currentSpeed < _targetSpeed) {
            _currentSpeed += _accel;
            if (_currentSpeed > _targetSpeed) _currentSpeed = _targetSpeed;
        } else if (_currentSpeed > _targetSpeed) {
            _currentSpeed -= _accel;
            if (_currentSpeed < _targetSpeed) _currentSpeed = _targetSpeed;
        }
        
    } else {
        // --- MODUL POZIȚIE (Folosind Encoderul) ---
        long long currentPos = getActualPosition();
        long long error = _targetStep - currentPos;
        
        // Dacă ești suficient de aproape, oprește-te (evită tremuratul/oscilația)
        if (abs(error) <= 2) {
            _currentSpeed = 0;
            return;
        }

        // Calculează viteza necesară pentru a ajunge acolo (PID Proporțional)
        float desiredSpeed = (float)error * _Kp;
        
        // Nu depăși limita mecanică setată de tine
        if (desiredSpeed > _maxSpeed) desiredSpeed = _maxSpeed;
        if (desiredSpeed < -_maxSpeed) desiredSpeed = -_maxSpeed;
        
        // Accelerează/Decelerează curat către acea viteză
        if (_currentSpeed < desiredSpeed) {
            _currentSpeed += _accel;
            if (_currentSpeed > desiredSpeed) _currentSpeed = desiredSpeed;
        } else if (_currentSpeed > desiredSpeed) {
            _currentSpeed -= _accel;
            if (_currentSpeed < desiredSpeed) _currentSpeed = desiredSpeed;
        }
    }
}