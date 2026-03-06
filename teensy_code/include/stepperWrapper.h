#ifndef STEPPER_WRAPPER_H
#define STEPPER_WRAPPER_H

#include <Arduino.h>
#include "TeensyTimerTool.h"
#include "MT6835_encoder.h"

// Setări mecanice
#define MOTOR_PPR 6400.0f
#define DEG_TO_MOTOR_STEPS(deg) ((deg) * MOTOR_PPR / 360.0f)

class stepperWrapper{
public:
    stepperWrapper(uint8_t dir, uint8_t step, MT6835* encoder);
    
    void begin();
    void update(); // Asta trebuie chemată des în loop() (ex. la 5ms)

    // --- FUNCȚII PENTRU MODUL POZIȚIE (Control cu Encoder) ---
    void setTargetStep(long long target) { _targetStep = target; }
    void setCurrentPositionInSteps(long long steps);
    long long getSteps(); // Întoarce poziția reală a encoderului (în pași de motor)

    // --- FUNCȚII PENTRU MODUL VITEZĂ (Așa cum aveai înainte) ---
    void setTargetRPM(float rpm); 
    void setRunWithoutPosition(bool state) { _runWithoutPosition = state; }

    // --- TUNING (Setează accelerația și viteza) ---
    void setMaxRPM(float rpm);
    void setAccelerationRate(float accelRate); // Cât de repede accelerează

private:
    void tick(); // Funcția hardware la 50kHz
    long long getActualPosition(); // Combină logica de encoder vs intern

    uint8_t _dir, _step;
    TeensyTimerTool::PeriodicTimer _sTimer;
    MT6835* _encoder;
    
    bool _runWithoutPosition = false;

    volatile long long _internalSteps = 0; // Folosit dacă nu e encoder
    volatile long long _targetStep = 0;
    
    volatile float _currentSpeed = 0; // Steps per second (intern)
    float _targetSpeed = 0;           // Steps per second pt modul Viteză
    
    // --- PARAMETRI DE MIȘCARE ---
    float _maxSpeed = 35000.0f; // Limitator hardware
    float _accel = 200.0f;      // Rata de accelerare per update()
    float _Kp = 25.0f;          // Cât de agresiv urmărește poziția

    // --- VARIABILE PENTRU TICK HARDWARE ---
    volatile float _accumulator = 0;
    volatile bool _stepPulseActive = false;
};

#endif