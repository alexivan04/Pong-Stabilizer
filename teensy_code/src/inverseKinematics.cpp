#include "inverseKinematics.h"

// Constante PID (Acum poți mări puțin Kd datorită filtrului)
float Kp = 0.126 , Ki = 0.02, Kd = 1.6; 
float errX_int = 0, lastErrX = 0;
float errY_int = 0, lastErrY = 0;

// Filtru EMA pentru datele camerei
float filteredX = 0;
float filteredY = 0;
// 1.0 = Fără filtru. 0.1 = Foarte smooth, dar cu puțin delay. (0.15 e ideal)
const float ALPHA = 0.2; 

float calculateArmAngle(int i, float pitch, float roll) {
    float Zi = h0 + Rp * (sin(roll) * cos(servo_gamma[i]) + sin(pitch) * sin(servo_gamma[i]));
    float Z_target = Zi - L3;
    float X_offset = abs(Rb - Rp); 

    float D = sqrt(sq(X_offset) + sq(Z_target));
    if (D > (L1 + L2)) D = L1 + L2;

    float gamma_angle = atan2(Z_target, X_offset);
    float cosBeta = (sq(L1) + sq(D) - sq(L2)) / (2 * L1 * D);
    float beta = acos(constrain(cosBeta, -1.0, 1.0));

    return (gamma_angle + beta) * 180.0 / PI;
}

void computeMotorAngles(float targetX, float targetY, volatile float *motorAngles) {
    float rawX = 0;
    float rawY = 0;

    // Daca vedem bila, folosim datele. Daca nu, platforma revine lin la centru.
    if (ballData.is_found) {
        rawX = ballData.x;
        rawY = ballData.y;
    }

    // Aplicăm filtrul pentru a opri tremuratul PID-ului (Zgomotul Camerei)
    filteredX = (ALPHA * rawX) + ((1.0f - ALPHA) * filteredX);
    filteredY = (ALPHA * rawY) + ((1.0f - ALPHA) * filteredY);

    float errorX = filteredX - targetX;
    float errorY = filteredY - targetY;

    // Axa X (Roll)
    errX_int = constrain(errX_int + errorX, -50, 50);
    float roll = Kp * errorX + Ki * errX_int + Kd * (errorX - lastErrX);
    lastErrX = errorX;

    // Axa Y (Pitch)
    errY_int = constrain(errY_int + errorY, -50, 50);
    float pitch = Kp * errorY + Ki * errY_int + Kd * (errorY - lastErrY);
    lastErrY = errorY;

    pitch = constrain(pitch, -10.0, 10.0) * PI / 180.0;
    roll = constrain(roll, -10.0, 10.0) * PI / 180.0;

    motorAngles[0] = calculateArmAngle(0, pitch, roll);
    motorAngles[1] = calculateArmAngle(1, pitch, roll);
    motorAngles[2] = calculateArmAngle(2, pitch, roll);
    motorAngles[3] = calculateArmAngle(3, pitch, roll);
}
