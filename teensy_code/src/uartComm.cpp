#include "uartComm.h"

// Referenced from main.cpp
extern uint8_t pendingPattern;
extern bool    pendingPatternFlag;

static byte rxBuffer[32];
static uint8_t payloadSize = 0;

static void processPacket() {
    byte packetType = rxBuffer[0];

    if (packetType == PACKET_TYPE_BALL_DATA) {
        memcpy(&ballData, &rxBuffer[2], sizeof(BallData));
    }
    if (packetType == PACKET_TYPE_PID_UPDATE) {
        memcpy(&pidValues, &rxBuffer[2], sizeof(PIDValues));
    }
    if (packetType == PACKET_TYPE_START) {
        isStarted = true;
    }
    // Pi -> Teensy: manual pattern selection
    if (packetType == PACKET_TYPE_PATTERN_CHANGE && rxBuffer[1] >= 1) {
        pendingPattern     = rxBuffer[2];
        pendingPatternFlag = true;
    }
}

void checkSerial() {
    static enum {
        WAITING_FOR_START,
        READING_PACKET
    } state = WAITING_FOR_START;

    static uint8_t bytesRead = 0;

    while (Serial1.available()) {
        byte b = Serial1.read();

        if (state == WAITING_FOR_START) {
            if (b == START_BYTE) {
                bytesRead = 0;
                state = READING_PACKET;
            }
        }
        else if (state == READING_PACKET) {
            rxBuffer[bytesRead++] = b;

            if (bytesRead == 2) {
                payloadSize = rxBuffer[1];
            }

            if (bytesRead > 1 && bytesRead == (payloadSize + 4)) {
                if (rxBuffer[bytesRead - 1] == END_BYTE) {
                    byte receivedChecksum   = rxBuffer[bytesRead - 2];
                    byte calculatedChecksum = 0;
                    for (int i = 0; i < bytesRead - 2; i++) {
                        calculatedChecksum ^= rxBuffer[i];
                    }
                    if (calculatedChecksum == receivedChecksum) {
                        processPacket();
                    } else {
                        Serial.println("Checksum mismatch!");
                    }
                } else {
                    Serial.println("End byte mismatch!");
                }
                state = WAITING_FOR_START;
            }
        }
    }
}

void sendPatternChange(uint8_t patternIndex) {
    uint8_t payload_size = 1;
    byte checksum = PACKET_TYPE_PATTERN_CHANGE ^ payload_size ^ patternIndex;

    Serial1.write(START_BYTE);
    Serial1.write(PACKET_TYPE_PATTERN_CHANGE);
    Serial1.write(payload_size);
    Serial1.write(patternIndex);
    Serial1.write(checksum);
    Serial1.write(END_BYTE);
}

void sendTargetPos(float tx, float ty, float heightDeg) {
    uint8_t payload[12];
    memcpy(payload,     &tx, 4);
    memcpy(payload + 4, &ty, 4);
    memcpy(payload + 8, &heightDeg, 4);   // plate height setting, deg above the low stop
    uint8_t payload_size = 12;

    byte checksum = PACKET_TYPE_TARGET_POS ^ payload_size;
    for (int i = 0; i < 12; i++) checksum ^= payload[i];

    Serial1.write(START_BYTE);
    Serial1.write(PACKET_TYPE_TARGET_POS);
    Serial1.write(payload_size);
    Serial1.write(payload, 12);
    Serial1.write(checksum);
    Serial1.write(END_BYTE);
}