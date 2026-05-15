#ifndef UART_COMM_H
#define UART_COMM_H

#include <Arduino.h>

// --- COMMUNICATION PROTOCOL ---
const byte START_BYTE = 0xAA;
const byte END_BYTE = 0xBB;
const byte PACKET_TYPE_BALL_DATA = 1;
const byte PACKET_TYPE_PID_UPDATE = 2;
const byte PACKET_TYPE_START = 3;
const byte PACKET_TYPE_PATTERN_CHANGE = 4;  // NEW: Teensy -> Pi

struct __attribute__((packed)) BallData {
  uint8_t is_found;
  float x;
  float y;
  float z;
};

struct __attribute__((packed)) PIDValues {
    float P;
    float I;
    float D;
};

extern BallData ballData;
extern PIDValues pidValues;
extern boolean isStarted;

void checkSerial();
void sendPatternChange(uint8_t patternIndex);  // NEW

#endif // UART_COMM_H