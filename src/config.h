#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <ESP8266WiFi.h>

// ========== 硬件引脚 ==========
#define STATUS_LED_PIN   2       // GPIO2, 低电平点亮
#define SERIAL_BAUD      115200  // 串口波特率，与 MCU UART6 一致

// ========== 时间参数（毫秒） ==========
#define WIFI_CHECK_INTERVAL   5000   // WiFi 断线检查间隔
#define HEARTBEAT_INTERVAL    15000  // 无变化时心跳间隔（状态变化则立即上报）
#define STATUS_TIMEOUT_MS     10000  // MCU 超时判定

// ========== 帧协议常量（MCU ↔ ESP 纯二进制） ==========
// 帧格式: [0xAA] [type] [len_hi] [len_lo] [payload N字节] [checksum]
// 校验和 = XOR(type, len_hi, len_lo, payload所有字节)
#define WIFI_FRAME_SYNC         0xAA
#define WIFI_FRAME_MAX_PAYLOAD  64
#define WIFI_FRAME_MAX_TOTAL    70

// 下行: ESP → MCU
#define WIFI_TYPE_SET_PANEL     0x20  // payload: panel_id(1B) + mode(1B)
#define WIFI_TYPE_SELFTEST      0x21  // payload: 空
#define WIFI_TYPE_TASK_SWITCH   0x22  // payload: task_id(1B)
#define WIFI_TYPE_CMD_INDEX     0x23  // payload: index(1B)
#define WIFI_TYPE_SHUTDOWN      0x24  // payload: 空

// 上行: MCU → ESP
#define WIFI_TYPE_PANEL_STATUS  0x11  // payload: 4×(state 1B + ts 4B LE) = 20B
#define WIFI_TYPE_FAULT         0x12  // payload: code(1B)
#define WIFI_TYPE_SELFTEST_R    0x13  // payload: ok(1B) + total(1B)
#define WIFI_TYPE_ACK           0x14  // payload: cmd(1B) + id(1B) + mode(1B) + ok(1B)

// 面板状态码（与 MCU 一致）
#define PANEL_STATE_IDLE        0x00
#define PANEL_STATE_CLEANING    0x01
#define PANEL_STATE_RESETTING   0x02

// 面板模式码（SET_PANEL/ACK payload 中用，与 MCU 一致）
#define PANEL_MODE_CLEAN   0x01
#define PANEL_MODE_RESET   0x02

// 动作码（与服务器指令兼容）
#define ACT_CLEAN  1
#define ACT_RESET  2
#define ACT_STOP   0

// ========== 服务器→ESP 二进制快速指令 ==========
// 5 字节: [0xAB] [dev_id] [mode] [0xCD] [checksum]
// checksum = XOR(dev_id, mode, 0xCD)
#define BIN_CMD_SYNC  0xAB
#define BIN_CMD_TAIL  0xCD
#define BIN_CMD_LEN   5

// ========== 调试输出开关 ==========
#define DEBUG_PRINT

// ========== 设备配置 ==========
struct DeviceConfig {
    String ssid;
    String password;
    IPAddress serverIP;
    uint16_t serverPort;
    bool useUDP;
    uint8_t deviceID;     // 本机设备号 1-4，每个光伏设备一个 ESP-12F
    bool     useStaticIP; // true=固定IP, false=DHCP
    IPAddress localIP;    // 本机固定 IP
    IPAddress gateway;    // 网关
    IPAddress subnet;     // 子网掩码
};

const DeviceConfig DEFAULT_CONFIG = {
    "1",
    "11111111",
    IPAddress(192, 168, 0, 255),
    4399,
    true,
    2,
    true,                                   // useStaticIP — 启用固定 IP
    IPAddress(192, 168, 0, 51),             // localIP
    IPAddress(192, 168, 0, 1),               // gateway
    IPAddress(255, 255, 255, 0)              // subnet
};

// ========== 设备状态 ==========
struct SensorData {
    int8_t   stepperMode;   // 0=空闲, 1=清洁中, 2=复位中
    bool     online;
    uint32_t lastUpdate;
};

// ========== 通信统计 ==========
struct CommStats {
    uint32_t framesSent;
    uint32_t framesRecv;
    uint32_t badFrames;
    uint32_t serverSent;
    uint32_t serverRecv;
};

#endif
