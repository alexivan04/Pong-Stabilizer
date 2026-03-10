#include "uartComm.h"

// Internal variables for the serial parser (hidden from other files using 'static')
static byte rxBuffer[32];
static uint8_t payloadSize = 0;

// Internal function to process a complete and valid packet
static void processPacket() {
    byte packetType = rxBuffer[0];
  
    if (packetType == PACKET_TYPE_BALL_DATA) {
        // Directly copy the payload from the buffer into our struct
        // This is very efficient.
        memcpy(&ballData, &rxBuffer[2], sizeof(BallData));
    }

    if (packetType == PACKET_TYPE_PID_UPDATE) {
        memcpy(&pidValues, &rxBuffer[2], sizeof(PIDValues));
    }
}

// Non-blocking function to check for and parse incoming packets
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

      // First byte after start is packet type, second is payload size
      if (bytesRead == 2) {
        payloadSize = rxBuffer[1];
      }

      // Check if we have received the full packet
      // Total length = type(1) + size(1) + payload(N) + checksum(1) + end(1)
      if (bytesRead > 1 && bytesRead == (payloadSize + 4)) {
        if (rxBuffer[bytesRead - 1] == END_BYTE) {
          // Verify checksum
          byte receivedChecksum = rxBuffer[bytesRead - 2];
          byte calculatedChecksum = 0;
          for (int i = 0; i < bytesRead - 2; i++) {
            calculatedChecksum ^= rxBuffer[i];
          }

          if (calculatedChecksum == receivedChecksum) {
            processPacket(); // Packet is valid!
          } else {
             Serial.println("Checksum mismatch!");
          }
        } else {
           Serial.println("End byte mismatch!");
        }
        state = WAITING_FOR_START; // Reset for next packet
      }
    }
  }
}
