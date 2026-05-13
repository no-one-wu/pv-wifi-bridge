#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <ESP8266WiFi.h>

// ========== Hardware pins ==========
#define STATUS_LED_PIN   2       // GPIO2, active LOW (LOW=connected)
#define SERIAL_BAUD      115200  // UART baud rate (matches MCU UART6)

// ========== Timing constants (milliseconds) ==========
#define WIFI_CHECK_INTERVAL   5000    // WiFi reconnect check interval
#define STATUS_SEND_INTERVAL  2000    // Status report interval to server
#define STATUS_TIMEOUT_MS     10000   // MCU offline timeout (no data for 10s)

// ========== 11-byte frame field constants ==========
#define FRAME_LEN          11
#define FRAME_HDR_BYTE0    0x00
#define FRAME_HDR_BYTE1    0x00
#define FRAME_HDR_BYTE2    0x06
#define FRAME_SYNC         0x55
#define FRAME_CMD_MOTOR    0xBC
#define FRAME_RESERVED     0x01   // byte[6] fixed value in MCU protocol
#define FRAME_END_MARKER   0xBB

// ========== Frame byte offsets ==========
#define FRAME_OFF_SYNC      3
#define FRAME_OFF_CMD       4
#define FRAME_OFF_ADDR      5
#define FRAME_OFF_RESERVED  6
#define FRAME_OFF_CLEAN     7
#define FRAME_OFF_RESET     8
#define FRAME_OFF_CHECKSUM  9
#define FRAME_OFF_END       10

// ========== Stepper mode constants (matching MCU Stepper_mode) ==========
#define MODE_IDLE     0
#define MODE_CLEANING 1
#define MODE_RESET    2

// ========== Server command action codes ==========
#define ACT_CLEAN  1
#define ACT_RESET  2
#define ACT_STOP   0

// ========== Configuration struct ==========
struct DeviceConfig {
    String ssid;
    String password;
    IPAddress serverIP;
    uint16_t serverPort;
    bool useUDP;
    uint8_t deviceID;
};
//wifi配置信息
const DeviceConfig DEFAULT_CONFIG = {
    "1",                              // ssid
    "12345678",                        // password
    IPAddress(192, 168, 0, 200),       // serverIP
    9000,                               // serverPort
    false,                              // useUDP (false = TCP)
    1                                   // deviceID (1-4)
};

// ========== Device status struct ==========
struct SensorData {
    int8_t   stepperMode;     // 0=idle, 1=cleaning, 2=reset
    bool     online;          // received data from MCU recently
    uint32_t lastUpdate;      // millis() of last valid MCU frame
};

// ========== Communication statistics ==========
struct CommStats {
    uint32_t framesSent;      // 11-byte frames sent to MCU
    uint32_t framesRecv;      // valid 11-byte frames received from MCU
    uint32_t badFrames;       // invalid/corrupt frames
    uint32_t serverSent;      // JSON status messages sent to server
    uint32_t serverRecv;      // JSON commands received from server
};

#endif
