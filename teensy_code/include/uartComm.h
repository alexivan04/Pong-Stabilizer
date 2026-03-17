#ifndef UART_COMM_H
#define UART_COMM_H

#include <Arduino.h>

// --- COMMUNICATION PROTOCOL ---
const byte START_BYTE = 0xAA;
const byte END_BYTE = 0xBB;
const byte PACKET_TYPE_BALL_DATA = 1;
const byte PACKET_TYPE_PID_UPDATE = 2;
const byte PACKET_TYPE_START = 3;
// This struct defines the exact layout of the payload for ball data.
// The __attribute__((packed)) is important to prevent memory alignment issues.
struct __attribute__((packed)) BallData {
  uint8_t is_found; // 0 for false, 1 for true
  float x;
  float y;
  float z;
};

struct __attribute__((packed)) PIDValues {
    float P;
    float I;
    float D;
};

// Declare the global variable so any file including this header can read the data.
extern BallData ballData;
extern PIDValues pidValues;
extern boolean isStarted;

// Function declarations
void checkSerial();

#endif // UART_COMM_H
