#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include "config.h"

// ========== Receive state machine states ==========
enum RxState {
    RX_IDLE,        // Waiting for first 0x00
    RX_HDR1,        // Got 0x00, waiting for second 0x00
    RX_HDR2,        // Got 0x00 0x00, waiting for 0x06
    RX_SYNC,        // Got 0x00 0x00 0x06, waiting for 0x55
    RX_CMD,         // Got sync, waiting for 0xBC
    RX_ADDR,        // Got 0xBC, reading address byte[5]
    RX_RSVD,        // Reading reserved byte[6]
    RX_DATA1,       // Reading byte[7] (clean flag / sensor data)
    RX_DATA2,       // Reading byte[8] (reset flag / sensor data)
    RX_CHECKSUM,    // Reading checksum byte[9]
    RX_END          // Waiting for 0xBB
};

static RxState      rxState = RX_IDLE;
static uint8_t      rxFrame[FRAME_LEN];
static uint8_t      rxPos = 0;

// ========== Calculate checksum for frame bytes[5..8] ==========
// Checksum = (ADDR + RESERVED + CLEAN + RESET) % 256
static uint8_t calcFrameChecksum(const uint8_t *frame) {
    uint16_t sum = (uint16_t)frame[FRAME_OFF_ADDR]
                 + (uint16_t)frame[FRAME_OFF_RESERVED]
                 + (uint16_t)frame[FRAME_OFF_CLEAN]
                 + (uint16_t)frame[FRAME_OFF_RESET];
    return (uint8_t)(sum & 0xFF);
}

// ========== Build an 11-byte command frame ==========
// devID: 1-4,  action: ACT_CLEAN or ACT_RESET
static void buildCommandFrame(uint8_t *frame, uint8_t devID, uint8_t action) {
    frame[0]  = FRAME_HDR_BYTE0;   // 0x00
    frame[1]  = FRAME_HDR_BYTE1;   // 0x00
    frame[2]  = FRAME_HDR_BYTE2;   // 0x06
    frame[3]  = FRAME_SYNC;        // 0x55
    frame[4]  = FRAME_CMD_MOTOR;   // 0xBC
    frame[5]  = devID;             // device address 1-4
    frame[6]  = FRAME_RESERVED;    // 0x01
    frame[7]  = (action == ACT_CLEAN) ? 0x01 : 0x00;
    frame[8]  = (action == ACT_RESET) ? 0x01 : 0x00;
    frame[9]  = calcFrameChecksum(frame);
    frame[10] = FRAME_END_MARKER;  // 0xBB
}

// ========== Send a command frame to MCU via Serial ==========
static void sendCommandToMCU(uint8_t devID, uint8_t action) {
    uint8_t frame[FRAME_LEN];
    buildCommandFrame(frame, devID, action);
    Serial.write(frame, FRAME_LEN);
}


// ========== Feed one byte into the receive state machine ==========
// Returns true if a complete valid frame was received (frame data in outFrame)
static bool parseMCUFrame(uint8_t byte, uint8_t *outFrame) {
    switch (rxState) {
        case RX_IDLE:
            if (byte == FRAME_HDR_BYTE0) {
                rxFrame[0] = byte;
                rxPos = 1;
                rxState = RX_HDR1;
            }
            break;

        case RX_HDR1:
            if (byte == FRAME_HDR_BYTE1) {
                rxFrame[1] = byte;
                rxPos = 2;
                rxState = RX_HDR2;
            } else {
                rxState = RX_IDLE;
            }
            break;

        case RX_HDR2:
            if (byte == FRAME_HDR_BYTE2) {
                rxFrame[2] = byte;
                rxPos = 3;
                rxState = RX_SYNC;
            } else {
                rxState = RX_IDLE;
            }
            break;

        case RX_SYNC:
            if (byte == FRAME_SYNC) {
                rxFrame[3] = byte;
                rxPos = 4;
                rxState = RX_CMD;
            } else {
                rxState = RX_IDLE;
            }
            break;

        case RX_CMD:
            if (byte == FRAME_CMD_MOTOR) {
                rxFrame[4] = byte;
                rxPos = 5;
                rxState = RX_ADDR;
            } else {
                rxState = RX_IDLE;
            }
            break;

        case RX_ADDR:
            rxFrame[5] = byte;
            rxPos = 6;
            rxState = RX_RSVD;
            break;

        case RX_RSVD:
            rxFrame[6] = byte;
            rxPos = 7;
            rxState = RX_DATA1;
            break;

        case RX_DATA1:
            rxFrame[7] = byte;
            rxPos = 8;
            rxState = RX_DATA2;
            break;

        case RX_DATA2:
            rxFrame[8] = byte;
            rxPos = 9;
            rxState = RX_CHECKSUM;
            break;

        case RX_CHECKSUM:
            rxFrame[9] = byte;
            rxPos = 10;
            rxState = RX_END;
            break;

        case RX_END:
            rxFrame[10] = byte;
            if (byte == FRAME_END_MARKER) {
                uint8_t expectedCk = calcFrameChecksum(rxFrame);
                if (rxFrame[FRAME_OFF_CHECKSUM] == expectedCk) {
                    memcpy(outFrame, rxFrame, FRAME_LEN);
                    rxState = RX_IDLE;
                    rxPos = 0;
                    return true;
                }
            }
            rxState = RX_IDLE;
            rxPos = 0;
            break;
    }
    return false;
}

// ========== Extract motor status from a received frame ==========
// Motor status frames have ADDR 1-4 (matching drivers_id)
// frame[7]=1/0 (cleaning), frame[8]=1/0 (reset) → derive Stepper_mode
// Returns true if data was updated
static bool extractSensorData(const uint8_t *frame, SensorData &data,
                               uint8_t localDevID) {
    if (frame[FRAME_OFF_CMD] != FRAME_CMD_MOTOR) return false;

    uint8_t addr = frame[FRAME_OFF_ADDR];
    if (addr < 1 || addr > 4) return false;
    if (addr != localDevID) return false;

    uint8_t d1 = frame[FRAME_OFF_CLEAN];   // byte[7]
    uint8_t d2 = frame[FRAME_OFF_RESET];   // byte[8]

    if (d1 == 1 && d2 == 0)
        data.stepperMode = MODE_CLEANING;
    else if (d1 == 0 && d2 == 1)
        data.stepperMode = MODE_RESET;
    else
        data.stepperMode = MODE_IDLE;

    data.lastUpdate = millis();
    data.online = true;
    return true;
}

#endif
