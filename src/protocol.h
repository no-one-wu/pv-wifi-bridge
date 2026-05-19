#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include "config.h"

/*
 * MCU ↔ ESP 纯二进制帧协议
 * 帧格式: [0xAA] [type] [len_hi] [len_lo] [payload N字节] [checksum]
 * 校验和 = XOR(type, len_hi, len_lo, payload所有字节)
 */

// ========== 帧接收状态机 ==========
enum MCURxState {
    MCU_RX_SYNC,
    MCU_RX_TYPE,
    MCU_RX_LEN_HI,
    MCU_RX_LEN_LO,
    MCU_RX_PAYLOAD,
    MCU_RX_CHECKSUM
};

static MCURxState  mcuRxState = MCU_RX_SYNC;
static uint8_t     mcuRxType = 0;
static uint16_t    mcuRxLen = 0;
static uint16_t    mcuRxIdx = 0;
static uint8_t     mcuRxCk = 0;
static uint8_t     mcuRxPayload[WIFI_FRAME_MAX_PAYLOAD];

// ========== buildBinaryFrame — 组装二进制帧（ESP→MCU） ==========
// out 至少 plen+5 字节，返回总字节数 = 5 + plen
static uint16_t buildBinaryFrame(uint8_t type, const uint8_t *payload,
                                  uint16_t plen, uint8_t *out) {
    out[0] = WIFI_FRAME_SYNC;
    out[1] = type;
    out[2] = (plen >> 8) & 0xFF;
    out[3] = plen & 0xFF;
    memcpy(out + 4, payload, plen);

    uint8_t ck = type ^ out[2] ^ out[3];
    for (uint16_t i = 0; i < plen; i++) ck ^= payload[i];
    out[4 + plen] = ck;

    return 5 + plen;
}

// ========== sendFrameToMCU — 向 MCU 发送二进制帧 ==========
static void sendFrameToMCU(uint8_t type, const uint8_t *payload, uint16_t plen) {
    uint8_t frame[WIFI_FRAME_MAX_TOTAL];
    uint16_t total = buildBinaryFrame(type, payload, plen, frame);
    Serial.write(frame, total);
}

// ========== parseMCUBinaryFrame — 帧接收状态机（逐字节） ==========
// 返回 true 表示收到完整有效帧
static bool parseMCUBinaryFrame(uint8_t byte, uint8_t *outType,
                                 uint8_t *outPayload, uint16_t *outLen) {
    switch (mcuRxState) {
        case MCU_RX_SYNC:
            if (byte == WIFI_FRAME_SYNC) {
                mcuRxCk = 0;
                mcuRxIdx = 0;
                mcuRxState = MCU_RX_TYPE;
            }
            break;

        case MCU_RX_TYPE:
            mcuRxType = byte;
            mcuRxCk = byte;
            mcuRxState = MCU_RX_LEN_HI;
            break;

        case MCU_RX_LEN_HI:
            mcuRxLen = ((uint16_t)byte) << 8;
            mcuRxCk ^= byte;
            mcuRxState = MCU_RX_LEN_LO;
            break;

        case MCU_RX_LEN_LO:
            mcuRxLen |= byte;
            mcuRxCk ^= byte;
            if (mcuRxLen > WIFI_FRAME_MAX_PAYLOAD) {
                mcuRxState = MCU_RX_SYNC;
                break;
            }
            if (mcuRxLen == 0) {
                mcuRxState = MCU_RX_CHECKSUM;
            } else {
                mcuRxIdx = 0;
                mcuRxState = MCU_RX_PAYLOAD;
            }
            break;

        case MCU_RX_PAYLOAD:
            mcuRxPayload[mcuRxIdx++] = byte;
            mcuRxCk ^= byte;
            if (mcuRxIdx >= mcuRxLen)
                mcuRxState = MCU_RX_CHECKSUM;
            break;

        case MCU_RX_CHECKSUM:
            if (byte == mcuRxCk) {
                *outType = mcuRxType;
                memcpy(outPayload, mcuRxPayload, mcuRxLen);
                *outLen = mcuRxLen;
                mcuRxState = MCU_RX_SYNC;
                return true;
            }
            mcuRxState = MCU_RX_SYNC;
            break;
    }
    return false;
}

// ========== parseMCUPanelBinary — 解析面板状态帧 (0x11) ==========
// payload 格式: 4×(state 1B + ts 4B LE) = 20B
// 按 deviceID 取对应偏移的面板数据
static void parseMCUPanelBinary(const uint8_t *payload, uint16_t len,
                                 SensorData &data, uint8_t localDevID) {
    if (len < 20) return;
    uint16_t off = (localDevID - 1) * 5;   // 每个面板占 5 字节
    if (off > 15) return;

    uint8_t st = payload[off];
    if (st == PANEL_STATE_CLEANING)
        data.stepperMode = ACT_CLEAN;
    else if (st == PANEL_STATE_RESETTING)
        data.stepperMode = ACT_RESET;
    else
        data.stepperMode = ACT_STOP;

    data.lastUpdate = millis();
    data.online = true;
}

// ========== parseMCUAckBinary — 解析命令应答帧 (0x14) ==========
// payload 格式: cmd(1B) + id(1B) + mode(1B) + ok(1B) = 4B
static void parseMCUAckBinary(const uint8_t *payload, uint16_t len,
                               SensorData &data, uint8_t localDevID) {
    if (len < 4) return;
    uint8_t id   = payload[1];
    uint8_t mode = payload[2];
    uint8_t ok   = payload[3];

    if (id != localDevID) return;
    if (!ok) return;

    if (mode == PANEL_MODE_CLEAN)
        data.stepperMode = ACT_CLEAN;
    else if (mode == PANEL_MODE_RESET)
        data.stepperMode = ACT_RESET;
    else
        data.stepperMode = ACT_STOP;

    data.lastUpdate = millis();
    data.online = true;
}

#endif
