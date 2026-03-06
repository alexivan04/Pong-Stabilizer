#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

// #include "TeensyTimerTool.h"
#include "uartComm.h"

// Dimensiuni mecanice (modifică valorile cu cele reale)
const float L1 = 98.1;    // Lungime braț motor (mm)
const float L2 = 92.0;    // Lungime braț intermediar (mm)
const float L3 = 54.0;    // Lungime link sub platformă (mm)
const float Rb = 129.0;   // Raza bazei (centru - ax motor) (mm)
const float Rp = 110.0;    // Raza platformei (centru - prindere) (mm)
const float h0 = 100.0;   // Înălțimea neutră (mm)

const float servo_gamma[4] = {5*PI/4.0f, 3*PI/4.0f, PI/4.0f, 7*PI/4.0f};
extern volatile float motorAngles[4];

float calculateArmAngle(int i, float pitch, float roll);
void computeMotorAngles(float targetX, float targetY, volatile float *motorAngles);

#endif
