#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "protocol.h"

// ================================================================
//  全局变量
// ================================================================
DeviceConfig  cfg = DEFAULT_CONFIG;   // 设备配置
SensorData    sensorData;             // MCU 状态
CommStats     stats;                  // 通信统计
WiFiClient    tcpClient;              // TCP 客户端
WiFiUDP       udpClient;              // UDP 客户端

uint32_t lastWiFiCheck  = 0;          // 上次 WiFi 检查时刻
uint32_t lastStatusSent = 0;          // 上次向服务器上报状态时刻

bool    pendingCommand = false;       // 是否有待发给 MCU 的命令
uint8_t pendingAction  = 0;           // 待发动作: ACT_CLEAN=1 / ACT_RESET=2
int8_t  lastStepperMode = -1;         // 上一次的电机模式，检测变化立即上报

// ================================================================
//  setStatusLED — WiFi 状态指示灯
// ================================================================
// connected=true → GPIO4 低电平 → 灯亮
void setStatusLED(bool connected) {
    digitalWrite(STATUS_LED_PIN, connected ? LOW : HIGH);
}

// ================================================================
//  buildStatusJSON — 构造上报服务器的设备状态 JSON
// ================================================================
// 格式: {"dev":1,"mode":0,"rssi":-55,"ts":123456}
void buildStatusJSON(char *buf, size_t len) {
    snprintf(buf, len,
        "{\"dev\":%u,\"mode\":%d,\"rssi\":%d,\"ts\":%lu}",
        cfg.deviceID, sensorData.stepperMode,
        WiFi.RSSI(), millis());
}

// ================================================================
//  parseServerCommand — 解析服务器 JSON 指令
// ================================================================
// 支持格式:
//   {"t":"set_panel","id":1,"mode":1}  (推荐) mode: 1=清洁 2=复位
//   {"cmd":1,"dev":2,"act":1}          (兼容) act:  1=清洁 2=复位
// 匹配本地 deviceID 后设置 pendingCommand，在 loop 中转发给 MCU
void parseServerCommand(const char *json) {
    uint8_t dev = 0;
    uint8_t act = 0;

    if (strstr(json, "\"set_panel\"")) {
        const char *idPos = strstr(json, "\"id\":");
        const char *modePos = strstr(json, "\"mode\":");
        if (!idPos || !modePos) return;
        dev = (uint8_t)atoi(idPos + 5);
        act = (uint8_t)atoi(modePos + 7);
    } else {
        const char *devPos = strstr(json, "\"dev\":");
        const char *actPos = strstr(json, "\"act\":");
        if (!devPos || !actPos) return;
        dev = (uint8_t)atoi(devPos + 6);
        act = (uint8_t)atoi(actPos + 6);
    }

    if (dev < 1 || dev > 4 || dev != cfg.deviceID) return;

    if (act == ACT_CLEAN || act == ACT_RESET) {
        pendingAction = act;
        pendingCommand = true;
#ifdef DEBUG_PRINT
        Serial.printf("[CMD] server -> dev=%u act=%u\n", dev, act);
#endif
    }
}

// ================================================================
//  parseBinaryCommand — 解析服务器发来的 5 字节二进制快速指令
// ================================================================
// 格式: [0xAB] [dev_id] [mode] [0xCD] [checksum]
// 比 JSON 少 32 字节传输量，省去字符串解析，适合低延迟控制
static void parseBinaryCommand(const uint8_t *data, uint16_t len) {
    if (len < BIN_CMD_LEN) return;
    if (data[0] != BIN_CMD_SYNC || data[3] != BIN_CMD_TAIL) {
        Serial.printf("[CMD:BIN] bad header: %02X %02X\n", data[0], data[3]);
        return;
    }

    uint8_t dev  = data[1];
    uint8_t mode = data[2];
    uint8_t ck   = (dev ^ mode ^ BIN_CMD_TAIL) & 0xFF;
    if (data[4] != ck) {
        Serial.printf("[CMD:BIN] bad ck: got %02X want %02X\n", data[4], ck);
        return;
    }

    if (dev < 1 || dev > 4 || dev != cfg.deviceID) {
        Serial.printf("[CMD:BIN] dev=%u ignored (local=%u)\n", dev, cfg.deviceID);
        return;
    }
    if (mode != ACT_CLEAN && mode != ACT_RESET) return;

    pendingAction = mode;
    pendingCommand = true;
    Serial.printf("[CMD:BIN] dev=%u act=%u -> pending\n", dev, mode);
}

// ================================================================
//  processSerialCommand — 串口文本配置命令
// ================================================================
// 支持的命令（大小写敏感）:
//   SET SSID <名称>             修改 WiFi
//   SET PASS <密码>             修改密码
//   SET SERVER <IP>:<端口>      修改服务器地址
//   SET MODE TCP|UDP            切换传输协议
//   SET DEV <1-4>               修改本机设备号
//   STATUS                      查看运行状态
void processSerialCommand(const String &cmd) {
    String line = cmd;
    line.trim();
    if (line.length() == 0) return;

    if (line.startsWith("SET SSID ")) {
        cfg.ssid = line.substring(9);
        WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
        Serial.println("[CMD] SSID -> " + cfg.ssid);
        return;
    }
    if (line.startsWith("SET PASS ")) {
        cfg.password = line.substring(9);
        WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
        Serial.println("[CMD] password updated");
        return;
    }
    if (line.startsWith("SET SERVER ")) {
        String p = line.substring(11);
        int colon = p.indexOf(':');
        if (colon > 0) {
            cfg.serverIP.fromString(p.substring(0, colon));
            cfg.serverPort = (uint16_t)p.substring(colon + 1).toInt();
            Serial.printf("[CMD] server -> %s:%u\n",
                cfg.serverIP.toString().c_str(), cfg.serverPort);
        }
        return;
    }
    if (line.startsWith("SET MODE ")) {
        String m = line.substring(9);
        m.toUpperCase();
        cfg.useUDP = (m == "UDP");
        Serial.printf("[CMD] mode -> %s\n", cfg.useUDP ? "UDP" : "TCP");
        return;
    }
    if (line.startsWith("SET DEV ")) {
        cfg.deviceID = (uint8_t)line.substring(8).toInt();
        if (cfg.deviceID < 1) cfg.deviceID = 1;
        if (cfg.deviceID > 4) cfg.deviceID = 4;
        Serial.printf("[CMD] device ID -> %u\n", cfg.deviceID);
        return;
    }
    if (line == "STATUS") {
        Serial.printf("[STATUS] WiFi:%s RSSI:%d dev:%u mode:%d server:%s:%u %s\n",
            WiFi.isConnected() ? "OK" : "DOWN", WiFi.RSSI(), cfg.deviceID,
            sensorData.stepperMode, cfg.serverIP.toString().c_str(),
            cfg.serverPort, cfg.useUDP ? "UDP" : "TCP");
        Serial.printf("[STATS] MCU frames sent:%u recv:%u bad:%u | server sent:%u recv:%u\n",
            stats.framesSent, stats.framesRecv, stats.badFrames,
            stats.serverSent, stats.serverRecv);
        return;
    }
    Serial.println("[CMD] unknown: " + line);
}

// ================================================================
//  readSerialCommands — 串口文本行缓冲读取
// ================================================================
// 逐字符读取，遇到换行把整行交给 processSerialCommand
void readSerialCommands() {
    static String cmdBuf = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdBuf.length() > 0) { processSerialCommand(cmdBuf); cmdBuf = ""; }
        } else {
            cmdBuf += c;
        }
    }
}

// ================================================================
//  connectWiFi / checkWiFi — WiFi 连接管理
// ================================================================
void connectWiFi() {
    if (WiFi.isConnected()) return;
    Serial.printf("[WIFI] connecting to %s ...\n", cfg.ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
}

void checkWiFi() {
    if (millis() - lastWiFiCheck < WIFI_CHECK_INTERVAL) return;
    lastWiFiCheck = millis();

    // 双重检测: isConnected() 或已获取 IP 都视为在线
    bool hasIP = WiFi.localIP().isSet();
    if (WiFi.isConnected() || hasIP) {
        static bool wasConnected = false;
        if (!wasConnected) {
            Serial.printf("[WIFI] connected! IP: %s RSSI: %d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
            wasConnected = true;
        }
        setStatusLED(true);
        return;
    }

    setStatusLED(false);
    wl_status_t status = WiFi.status();
    const char* names[] = {
        "IDLE","NO_SSID_AVAIL","SCAN_COMPLETED",
        "CONNECTED","CONNECT_FAILED","CONNECTION_LOST","DISCONNECTED"
    };
    const char* name = (status <= WL_DISCONNECTED) ? names[status] : "UNKNOWN";

    // 异常情况: 路由器显示有 IP 但 SDK 说未连接
    if (hasIP) {
        Serial.printf("[WIFI] BUG? has IP=%s but status=%s\n",
            WiFi.localIP().toString().c_str(), name);
    }

    if (status == WL_IDLE_STATUS || status == WL_DISCONNECTED) {
        Serial.printf("[WIFI] status=%s, retrying...\n", name);
        WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
    } else if (status == WL_CONNECT_FAILED) {
        Serial.printf("[WIFI] CONNECT_FAILED (password/signal?)\n");
    } else if (status == WL_NO_SSID_AVAIL) {
        Serial.printf("[WIFI] NO_SSID_AVAIL (check 2.4GHz band)\n");
    } else {
        Serial.printf("[WIFI] status=%s, waiting...\n", name);
    }
}

// ================================================================
//  ensureServerConnected / sendToServer / receiveFromServer — 网络通信
// ================================================================
bool ensureServerConnected() {
    if (cfg.useUDP) return true;
    if (tcpClient.connected()) return true;
    if (tcpClient.connect(cfg.serverIP, cfg.serverPort)) {
        Serial.println("[NET] TCP connected");
        return true;
    }
    return false;
}

bool sendToServer(const char *data) {
    bool ok = false;
    size_t len = strlen(data);
    if (cfg.useUDP) {
        udpClient.beginPacket(cfg.serverIP, cfg.serverPort);
        udpClient.print(data);
        ok = (udpClient.endPacket() == 1);
    } else {
        if (ensureServerConnected()) {
            size_t sent = tcpClient.print(data);
            ok = (sent == len);
            if (!ok) tcpClient.stop();
        }
    }
    stats.serverSent++;
    return ok;
}

void receiveFromServer() {
    if (cfg.useUDP) {
        int pktSize = udpClient.parsePacket();
        if (pktSize > 0) {
            uint8_t buf[256];
            int n = udpClient.read(buf, sizeof(buf) - 1);
            if (n > 0) {
                stats.serverRecv++;
                buf[n] = 0;
                // 二进制快速通道: 首字节 0xAB → 5 字节指令
                if (n >= BIN_CMD_LEN && buf[0] == BIN_CMD_SYNC) {
#ifdef DEBUG_PRINT
                    Serial.printf("[DOWN:BIN] %u bytes\n", n);
#endif
                    parseBinaryCommand(buf, n);
                } else {
#ifdef DEBUG_PRINT
                    Serial.printf("[DOWN] %s\n", (char*)buf);
#endif
                    parseServerCommand((char*)buf);
                }
            }
        }
    } else {
        // TCP 模式: 先确认有足够数据再读，避免 peek 后 read 阻塞
        int avail = tcpClient.available();
        if (tcpClient.connected() && avail > 0) {
            int first = tcpClient.peek();
            if (first < 0) return;

            if ((uint8_t)first == BIN_CMD_SYNC && avail >= BIN_CMD_LEN) {
                // 二进制快速通道: 确认 5 字节都到了再一次性读走
                uint8_t bin[5];
                size_t n = tcpClient.read(bin, BIN_CMD_LEN);
                if (n >= BIN_CMD_LEN) {
                    stats.serverRecv++;
                    Serial.printf("[DOWN:BIN] dev=%u mode=%u\n", bin[1], bin[2]);
                    parseBinaryCommand(bin, n);
                }
            } else if ((uint8_t)first != BIN_CMD_SYNC) {
                // JSON 通道
                String line = tcpClient.readStringUntil('\n');
                if (line.length() > 0) {
                    stats.serverRecv++;
                    Serial.printf("[DOWN] %s\n", line.c_str());
                    parseServerCommand(line.c_str());
                }
            }
            // first==0xAB 但 avail<5: 等下一轮 loop 数据到齐
        }
    }
}

// ================================================================
//  processIncomingMCUData — 从串口读取 MCU 发来的二进制帧
// ================================================================
// peek 首字节判断:
//   0xAA → 二进制帧 (0xAA + type + len + JSON + checksum)，喂入状态机
//   其他  → 文本行，由 readSerialCommands 处理
// 收到完整帧后按类型分发:
//   type 0x11 (PANEL_STATUS): 调用 parseMCUPanelJSON 更新电机状态
//   type 0x14 (ACK):          打印应答确认
//   type 0x12 (FAULT):        打印故障信息
void processIncomingMCUData() {
    if (Serial.available() == 0) return;

    uint8_t firstByte = Serial.peek();

    if (firstByte == WIFI_FRAME_SYNC) {
        while (Serial.available()) {
            uint8_t b = Serial.read();
            uint8_t outType;
            uint8_t outPayload[WIFI_FRAME_MAX_PAYLOAD];
            uint16_t outLen;

            if (parseMCUBinaryFrame(b, &outType, outPayload, &outLen)) {
                stats.framesRecv++;
                outPayload[outLen] = '\0';   // null-terminate JSON

                switch (outType) {
                    case WIFI_TYPE_PANEL_STATUS:
                        // MCU 面板状态: {"t":"panels","p":[...]}
                        parseMCUPanelJSON((const char*)outPayload, sensorData, cfg.deviceID);
                        break;
                    case WIFI_TYPE_ACK:
                        // ACK 包含命令执行结果: {"t":"ack","cmd":"set_panel","id":2,"mode":1,"ok":1}
                        // 从 ACK 推断当前电机模式，不依赖 MCU 面板状态上报
                        {
                            const char *m = strstr((const char*)outPayload, "\"mode\":");
                            const char *o = strstr((const char*)outPayload, "\"ok\":");
                            if (m && o) {
                                int mode = atoi(m + 7);
                                int ok = atoi(o + 5);
                                if (ok && mode >= 1 && mode <= 2) {
                                    sensorData.stepperMode = (int8_t)mode;
                                    sensorData.lastUpdate = millis();
                                    sensorData.online = true;
                                    Serial.printf("[ACK] mode=%d ok=%d -> stepperMode updated\n", mode, ok);
                                }
                            }
                        }
                        break;
                    case WIFI_TYPE_FAULT:
                        Serial.printf("[FAULT] %s\n", outPayload);
                        break;
                    default:
                        Serial.printf("[MCU] type=0x%02X len=%u\n", outType, outLen);
                        break;
                }
            } else if (mcuRxState == MCU_RX_SYNC && b != WIFI_FRAME_SYNC) {
                stats.badFrames++;
            }
        }
    }
}

// ================================================================
//  processOutgoingCommands — 将待发命令打包成二进制帧发给 MCU
// ================================================================
// 服务器 JSON 已解析为 pendingAction (1=清洁 2=复位)
// 打包成 MCU 期望的格式: 二进制帧(0xAA+type=0x20+JSON负载)，直接发送
void processOutgoingCommands() {
    if (!pendingCommand) return;

    // 构造 MCU 期望的 JSON: {"t":"set_panel","id":N,"mode":M}
    char payload[64];
    snprintf(payload, sizeof(payload),
        "{\"t\":\"set_panel\",\"id\":%u,\"mode\":%u}",
        cfg.deviceID, pendingAction);

    sendFrameToMCU(WIFI_TYPE_SET_PANEL, payload, strlen(payload));
    stats.framesSent++;
#ifdef DEBUG_PRINT
    Serial.printf("[TXMCU] %s\n", payload);
#endif
    pendingCommand = false;
}

// ================================================================
//  periodicStatusReport — 向服务器上报设备状态
// ================================================================
// 两种触发:
//   1. 电机模式发生变化 → 立即上报
//   2. 每 STATUS_SEND_INTERVAL 定时心跳
void periodicStatusReport() {
    bool modeChanged = (sensorData.stepperMode != lastStepperMode);
    bool heartbeatDue = (millis() - lastStatusSent >= STATUS_SEND_INTERVAL);

    if (!modeChanged && !heartbeatDue) return;
    if (!WiFi.isConnected()) return;

    char json[256];
    buildStatusJSON(json, sizeof(json));

    if (sendToServer(json)) {
        lastStepperMode = sensorData.stepperMode;
        lastStatusSent = millis();
#ifdef DEBUG_PRINT
        Serial.printf("[SEND] %s\n", json);
#endif
    } else {
        Serial.printf("[SEND] FAIL %s\n", json);
    }
}

// ================================================================
//  handleWatchdog — 喂硬件看门狗
// ================================================================
void handleWatchdog() { ESP.wdtFeed(); }

// ================================================================
//  setup — 上电初始化
// ================================================================
void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    setStatusLED(false);

    Serial.begin(SERIAL_BAUD);
    while (!Serial) ;
    Serial.setTimeout(10);

    Serial.println("\n[BOOT] ESP-12F PV WiFi Bridge v1.1");
    Serial.printf("[BOOT] dev=%u server=%s:%u %s\n",
        cfg.deviceID, cfg.serverIP.toString().c_str(),
        cfg.serverPort, cfg.useUDP ? "UDP" : "TCP");

    memset(&sensorData, 0, sizeof(sensorData));
    sensorData.stepperMode = MODE_IDLE;

    connectWiFi();
    // setStatusLED(true);
    if (!cfg.useUDP) tcpClient.setNoDelay(true);

    lastWiFiCheck  = millis();
    lastStatusSent = millis();
}

// ================================================================
//  loop — 主循环 (每轮 ~10ms)
// ================================================================
//  1. 喂狗
//  2. 处理 MCU 串口数据（二进制帧优先，文本其次）
//  3. WiFi 断线检查
//  4. 接收服务器指令
//  5. 转发命令给 MCU
//  6. 定时上报状态
//  7. MCU 超时检测
void loop() {
    handleWatchdog();

    processIncomingMCUData();

    if (Serial.available()) {
        uint8_t firstByte = Serial.peek();
        if (firstByte != WIFI_FRAME_SYNC) readSerialCommands();
    }

    checkWiFi();
    receiveFromServer();
    processOutgoingCommands();
    periodicStatusReport();

    if (sensorData.online &&
        (millis() - sensorData.lastUpdate > STATUS_TIMEOUT_MS)) {
        sensorData.online = false;
        Serial.println("[WARN] MCU timeout (10s)");
    }

    delay(10);
}
