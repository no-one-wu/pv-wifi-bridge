#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include "config.h"

/*
 * ESP-12F 与 GD32F407 MCU 之间的二进制帧协议
 * 与 MCU 侧 wifi_driver.c 的帧格式逐字节一致：
 *   [0xAA] [type] [len_hi] [len_lo] [JSON payload N字节] [checksum]
 *   校验和 = type ^ len_hi ^ len_lo ^ payload[0] ^ ... ^ payload[N-1]
 */

// ========== MCU 帧接收状态机 ==========
enum MCURxState {
    MCU_RX_SYNC,        // 等待帧同步字 0xAA
    MCU_RX_TYPE,        // 读取帧类型
    MCU_RX_LEN_HI,      // 读取负载长度高字节
    MCU_RX_LEN_LO,      // 读取负载长度低字节
    MCU_RX_PAYLOAD,     // 读取 JSON 负载
    MCU_RX_CHECKSUM     // 读取校验字节，验证
};

static MCURxState  mcuRxState = MCU_RX_SYNC;
static uint8_t     mcuRxType = 0;
static uint16_t    mcuRxLen = 0;
static uint16_t    mcuRxIdx = 0;
static uint8_t     mcuRxCk = 0;           // 累计算的校验和
static uint8_t     mcuRxPayload[WIFI_FRAME_MAX_PAYLOAD];

// ================================================================
//  buildBinaryFrame — 组装二进制帧（ESP→MCU）
// ================================================================
// 参数:
//   type    — 帧类型码（如 WIFI_TYPE_SET_PANEL=0x20）
//   payload — JSON 字符串
//   plen    — JSON 字符串长度
//   out     — 输出缓冲区（调用者分配，至少 plen+5 字节）
// 返回: 完整帧的总字节数（= 5 + plen）
// 帧格式: [0xAA] [type] [len_hi] [len_lo] [payload...] [checksum]
static uint16_t buildBinaryFrame(uint8_t type, const char *payload,
                                  uint16_t plen, uint8_t *out) {
    out[0] = WIFI_FRAME_SYNC;
    out[1] = type;
    out[2] = (plen >> 8) & 0xFF;      // 负载长度高字节（大端）
    out[3] = plen & 0xFF;             // 负载长度低字节
    memcpy(out + 4, payload, plen);

    // 校验和 = XOR(type, len_hi, len_lo, 每个负载字节)
    uint8_t ck = type ^ out[2] ^ out[3];
    for (uint16_t i = 0; i < plen; i++) ck ^= (uint8_t)payload[i];
    out[4 + plen] = ck;

    return 5 + plen;   // 帧头(1)+类型(1)+长度(2)+负载(N)+校验(1)
}

// ================================================================
//  sendFrameToMCU — 向 MCU 发送二进制帧
// ================================================================
// 参数:
//   type    — 帧类型码
//   payload — JSON 字符串
//   plen    — JSON 字符串长度（不含 '\0'）
// 内部调 buildBinaryFrame 拼帧，再 Serial.write 发出
static void sendFrameToMCU(uint8_t type, const char *payload, uint16_t plen) {
    uint8_t frame[WIFI_FRAME_MAX_TOTAL];
    uint16_t total = buildBinaryFrame(type, payload, plen, frame);
    Serial.write(frame, total);
}

// ================================================================
//  parseMCUBinaryFrame — MCU 二进制帧接收状态机（逐字节输入）
// ================================================================
// 参数:
//   byte       — 从串口读入的单个字节
//   outType    — 输出: 帧类型码
//   outPayload — 输出: JSON 负载缓冲区（调用者分配，至少 WIFI_FRAME_MAX_PAYLOAD 字节）
//   outLen     — 输出: JSON 负载长度
// 返回值: true=收到完整有效帧，false=尚未完成或校验失败
//
// 状态流程:
//   MCU_RX_SYNC → MCU_RX_TYPE → MCU_RX_LEN_HI → MCU_RX_LEN_LO
//   → MCU_RX_PAYLOAD → MCU_RX_CHECKSUM → 回到 MCU_RX_SYNC
// 任一步骤失败都直接回到 MCU_RX_SYNC
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
            mcuRxCk = byte;              // 校验和从 type 开始累加
            mcuRxState = MCU_RX_LEN_HI;
            break;

        case MCU_RX_LEN_HI:
            mcuRxLen = ((uint16_t)byte) << 8;  // 长度高字节
            mcuRxCk ^= byte;
            mcuRxState = MCU_RX_LEN_LO;
            break;

        case MCU_RX_LEN_LO:
            mcuRxLen |= byte;                  // 长度低字节
            mcuRxCk ^= byte;
            if (mcuRxLen > WIFI_FRAME_MAX_PAYLOAD) {
                mcuRxState = MCU_RX_SYNC;      // 长度非法，丢弃
                break;
            }
            if (mcuRxLen == 0) {
                mcuRxState = MCU_RX_CHECKSUM;  // 空负载直接等校验
            } else {
                mcuRxIdx = 0;
                mcuRxState = MCU_RX_PAYLOAD;
            }
            break;

        case MCU_RX_PAYLOAD:
            mcuRxPayload[mcuRxIdx++] = byte;
            mcuRxCk ^= byte;                   // 每个负载字节参与 XOR
            if (mcuRxIdx >= mcuRxLen) {
                mcuRxState = MCU_RX_CHECKSUM;
            }
            break;

        case MCU_RX_CHECKSUM:
            if (byte == mcuRxCk) {
                // 校验通过：拷贝结果到调用者缓冲区
                *outType = mcuRxType;
                memcpy(outPayload, mcuRxPayload, mcuRxLen);
                *outLen = mcuRxLen;
                mcuRxState = MCU_RX_SYNC;
                return true;
            }
            // 校验失败
            mcuRxState = MCU_RX_SYNC;
            break;
    }
    return false;
}

// ================================================================
//  parseMCUPanelJSON — 从 MCU 面板状态 JSON 中提取本设备状态
// ================================================================
// MCU 每秒发送 type=0x11 帧，JSON 格式:
//   {"t":"panels","p":[{"id":1,"st":0,"ts":0},{"id":2,"st":0,"ts":0},...]}
// 参数:
//   json      — JSON 字符串（来自帧负载）
//   data      — 设备状态（更新 stepperMode/online/lastUpdate）
//   localDevID — 本机设备号 1-4，只提取匹配的面板条目
// st 含义: 0=空闲, 1=清洁中, 2=复位中
static void parseMCUPanelJSON(const char *json, SensorData &data,
                               uint8_t localDevID) {
    // 构造搜索串 "\"id\":N" 定位本设备条目
    char search[16];
    snprintf(search, sizeof(search), "\"id\":%u", localDevID);

    const char *idPos = strstr(json, search);
    if (!idPos) return;

    const char *stPos = strstr(idPos, "\"st\":");
    if (!stPos) return;

    int st = atoi(stPos + 5);      // 跳过 "st":
    if (st == MODE_CLEANING)
        data.stepperMode = MODE_CLEANING;
    else if (st == MODE_RESET)
        data.stepperMode = MODE_RESET;
    else
        data.stepperMode = MODE_IDLE;

    data.lastUpdate = millis();
    data.online = true;
}

#endif
