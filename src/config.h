#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <ESP8266WiFi.h>

// ========== 硬件引脚 ==========
#define STATUS_LED_PIN   2       // GPIO4, 低电平点亮。不用GPIO2(启动引脚)
#define SERIAL_BAUD      115200  // 串口波特率，与 MCU UART6 一致

// ========== 时间参数（毫秒） ==========
#define WIFI_CHECK_INTERVAL   5000   // WiFi 断线检查间隔：5秒
#define HEARTBEAT_INTERVAL    15000  // 无变化时心跳间隔：15秒（状态变化则立即上报）
#define STATUS_TIMEOUT_MS     10000  // MCU 超时判定：10秒无数据视为离线

// ========== MCU 二进制帧协议常量 ==========
// 与 MCU 侧 wifi_driver.c 的帧格式完全一致：
//   [0xAA] [type] [len_hi] [len_lo] [payload N字节] [checksum]
//   校验和 = type ^ len_hi ^ len_lo ^ payload[0] ^ ... ^ payload[N-1]
#define WIFI_FRAME_SYNC       0xAA    // 帧同步字
#define WIFI_FRAME_MAX_PAYLOAD 512    // 单帧最大负载字节数（与 MCU 一致）
#define WIFI_FRAME_MAX_TOTAL   520    // 整帧最大字节数（5 + 512）

// MCU 定义的帧类型码（与 wifi_driver.h 保持一致）
#define WIFI_TYPE_PANEL_STATUS  0x11  // MCU→ESP: 面板状态上报
#define WIFI_TYPE_FAULT         0x12  // MCU→ESP: 故障告警
#define WIFI_TYPE_SELFTEST_R    0x13  // MCU→ESP: 自检结果
#define WIFI_TYPE_ACK           0x14  // MCU→ESP: 命令应答
#define WIFI_TYPE_SET_PANEL     0x20  // ESP→MCU: 设置面板模式
#define WIFI_TYPE_SELFTEST      0x21  // ESP→MCU: 自检请求
#define WIFI_TYPE_TASK_SWITCH   0x22  // ESP→MCU: 任务切换
#define WIFI_TYPE_CMD_INDEX     0x23  // ESP→MCU: 兼容旧 LoRa 指令索引
#define WIFI_TYPE_SHUTDOWN      0x24  // ESP→MCU: 系统关机

// MCU 电机模式常量（对应面板 st 字段）
#define MODE_IDLE     0   // 空闲
#define MODE_CLEANING 1   // 清洁（除尘）
#define MODE_RESET    2   // 复位

// 服务器指令动作码（对应 mode 字段，兼容新旧两种格式）
#define ACT_CLEAN  1      // 除尘
#define ACT_RESET  2      // 复位
#define ACT_STOP   0      // 停止

// ========== 服务器→ESP 二进制快速指令常量 ==========
// 5 字节极简协议，比 JSON 快 15-30ms，适合对延迟敏感的控制场景
// 帧格式: [0xAB] [dev_id] [mode] [0xCD] [checksum]
//   checksum = (dev_id ^ mode ^ 0xCD) & 0xFF
#define BIN_CMD_SYNC   0xAB   // 二进制指令帧头
#define BIN_CMD_TAIL   0xCD   // 二进制指令帧尾（兼命令类型标识）
#define BIN_CMD_LEN    5      // 二进制指令总字节数

// ========== 调试输出开关 ==========
// 注释此行关闭高频调试打印（[DOWN]/[SEND]/[TXMCU]），减少串口阻塞
#define DEBUG_PRINT

// ========== 设备配置结构体 ==========
struct DeviceConfig {
    String ssid;          // WiFi 名称（仅 2.4GHz）
    String password;      // WiFi 密码
    IPAddress serverIP;   // 远端服务器 IP
    uint16_t serverPort;  // 服务器端口
    bool useUDP;          // true=UDP, false=TCP（推荐）
    uint8_t deviceID;     // 本机设备号 1-4，只响应匹配的指令
};

// 默认配置，固件烧录后首次使用，可通过串口 SET 命令运行时修改
const DeviceConfig DEFAULT_CONFIG = {
    "401-iot",                       // ssid    iQOO Neo9 Pro
    "12345678",                           // password
    IPAddress(192,168,1,118),              // serverIP   192, 168, 157, 179
    9000,                                  // serverPort
    false,                                 // useUDP
    1                                      // deviceID（1-4）
};

// ========== 设备状态 ==========
struct SensorData {
    int8_t   stepperMode;   // 电机模式: 0=空闲, 1=清洁中, 2=复位中
    bool     online;        // MCU 是否在线（最近收到过有效帧）
    uint32_t lastUpdate;    // 最后一次收到 MCU 帧的时刻（millis()）
};

// ========== 通信统计 ==========
struct CommStats {
    uint32_t framesSent;    // 已向 MCU 发送的帧数
    uint32_t framesRecv;    // 已从 MCU 接收的有效帧数
    uint32_t badFrames;     // 校验失败/格式错误的帧数
    uint32_t serverSent;    // 已向服务器发送的 JSON 消息数
    uint32_t serverRecv;    // 已从服务器接收的指令数
};

#endif
