#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "protocol.h"

// ================================================================
//  全局变量
// ================================================================
DeviceConfig  cfg = DEFAULT_CONFIG;
SensorData    sensorData;
CommStats     stats;
WiFiClient    tcpClient;
WiFiUDP       udpClient;

uint32_t lastWiFiCheck  = 0;
uint32_t lastHeartbeat  = 0;

bool    pendingCommand = false;
uint8_t pendingAction  = 0;
int8_t  lastStepperMode = -1;

// ================================================================
//  setStatusLED — WiFi 状态指示灯
// ================================================================
void setStatusLED(bool connected) {
    digitalWrite(STATUS_LED_PIN, connected ? LOW : HIGH);
}

// ================================================================
//  buildStatusJSON — 构造上报服务器的设备状态 JSON
// ================================================================
void buildStatusJSON(char *buf, size_t len) {
    snprintf(buf, len,
        "{\"dev\":%u,\"mode\":%d,\"rssi\":%d,\"ts\":%lu}",
        cfg.deviceID, sensorData.stepperMode,
        WiFi.RSSI(), millis());
}

// ================================================================
//  parseServerCommand — 解析服务器 JSON 指令（不变）
// ================================================================
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
//  parseBinaryCommand — 解析服务器 5 字节二进制快速指令（不变）
// ================================================================
static void parseBinaryCommand(const uint8_t *data, uint16_t len) {
    if (len < BIN_CMD_LEN) return;
    if (data[0] != BIN_CMD_SYNC || data[3] != BIN_CMD_TAIL) return;

    uint8_t dev  = data[1];
    uint8_t mode = data[2];
    uint8_t ck   = (dev ^ mode ^ BIN_CMD_TAIL) & 0xFF;
    if (data[4] != ck) {
        Serial.printf("[CMD:BIN] bad ck: got %02X want %02X\n", data[4], ck);
        return;
    }

    if (dev < 1 || dev > 4 || dev != cfg.deviceID) return;
    if (mode != ACT_CLEAN && mode != ACT_RESET) return;

    pendingAction = mode;
    pendingCommand = true;
    Serial.printf("[CMD:BIN] dev=%u act=%u\n", dev, mode);
}

// ================================================================
//  processSerialCommand — 串口文本配置命令（不变）
// ================================================================
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
//  readSerialCommands — 串口文本行缓冲读取（不变）
// ================================================================
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
//  connectWiFi / checkWiFi — WiFi 连接管理（不变）
// ================================================================
void connectWiFi() {
    if (WiFi.isConnected()) return;
    Serial.printf("[WIFI] connecting to %s ...\n", cfg.ssid.c_str());
    WiFi.mode(WIFI_STA);
    if (cfg.useStaticIP) {
        WiFi.config(cfg.localIP, cfg.gateway, cfg.subnet);
    }
    WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
}

void checkWiFi() {
    if (millis() - lastWiFiCheck < WIFI_CHECK_INTERVAL) return;
    lastWiFiCheck = millis();

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
//  ensureServerConnected / sendToServer / receiveFromServer（不变）
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
        udpClient.write((const uint8_t*)data, len);
        ok = (udpClient.endPacket() == 1);
    } else {
        if (ensureServerConnected()) {
            size_t sent = tcpClient.write((const uint8_t*)data, len);
            tcpClient.flush();
            ok = (sent == len) && tcpClient.connected();
            if (!ok) {
                Serial.printf("[SEND] TCP fail, reconnecting...\n");
                tcpClient.stop();
            }
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
        int avail = tcpClient.available();
        if (tcpClient.connected() && avail > 0) {
            int first = tcpClient.peek();
            if (first < 0) return;

            if ((uint8_t)first == BIN_CMD_SYNC && avail >= BIN_CMD_LEN) {
                uint8_t bin[5];
                size_t n = tcpClient.read(bin, BIN_CMD_LEN);
                if (n >= BIN_CMD_LEN) {
                    stats.serverRecv++;
                    Serial.printf("[DOWN:BIN] dev=%u mode=%u\n", bin[1], bin[2]);
                    parseBinaryCommand(bin, n);
                }
            } else if ((uint8_t)first != BIN_CMD_SYNC) {
                String line = tcpClient.readStringUntil('\n');
                if (line.length() > 0) {
                    stats.serverRecv++;
                    Serial.printf("[DOWN] %s\n", line.c_str());
                    parseServerCommand(line.c_str());
                }
            }
        }
    }
}

// ================================================================
//  processIncomingMCUData — 从串口读取 MCU 发来的二进制帧
// ================================================================
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

                switch (outType) {
                    case WIFI_TYPE_PANEL_STATUS:
                        // 20B 二进制面板状态，4路面板按顺序排列
                        parseMCUPanelBinary(outPayload, outLen, sensorData, cfg.deviceID);
#ifdef DEBUG_PRINT
                        Serial.printf("[MCU] PANEL dev=%u st=%d\n",
                            cfg.deviceID, sensorData.stepperMode);
#endif
                        break;

                    case WIFI_TYPE_ACK:
                        // 4B ACK: cmd + id + mode + ok
                        parseMCUAckBinary(outPayload, outLen, sensorData, cfg.deviceID);
#ifdef DEBUG_PRINT
                        Serial.printf("[MCU] ACK id=%u mode=%u ok=%u -> stepperMode=%d\n",
                            outPayload[1], outPayload[2], outPayload[3], sensorData.stepperMode);
#endif
                        break;

                    case WIFI_TYPE_FAULT:
                        Serial.printf("[MCU] FAULT code=%u\n", outLen > 0 ? outPayload[0] : 0);
                        break;

                    case WIFI_TYPE_SELFTEST_R:
                        Serial.printf("[MCU] SELFTEST_R ok=%u total=%u\n",
                            outLen > 1 ? outPayload[0] : 0,
                            outLen > 1 ? outPayload[1] : 0);
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
// 构造纯二进制 payload: [panel_id 1B] [mode 1B]
void processOutgoingCommands() {
    if (!pendingCommand) return;

    uint8_t payload[2];
    payload[0] = (uint8_t)cfg.deviceID;
    payload[1] = pendingAction;   // ACT_CLEAN=1 / ACT_RESET=2

    sendFrameToMCU(WIFI_TYPE_SET_PANEL, payload, 2);
    stats.framesSent++;
#ifdef DEBUG_PRINT
    Serial.printf("[TXMCU] SET_PANEL id=%u mode=%u\n", payload[0], payload[1]);
#endif
    pendingCommand = false;
}

// ================================================================
//  periodicStatusReport — 向服务器上报设备状态（不变）
// ================================================================
void periodicStatusReport() {
    bool modeChanged = (sensorData.stepperMode != lastStepperMode);
    bool heartbeatDue = (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL);

    if (!modeChanged && !heartbeatDue) return;
    if (!WiFi.isConnected()) return;

    char json[256];
    buildStatusJSON(json, sizeof(json));

    if (sendToServer(json)) {
        lastStepperMode = sensorData.stepperMode;
        lastHeartbeat = millis();
    } else {
        Serial.printf("[SEND] FAIL %s\n", json);
    }
}

// ================================================================
//  handleWatchdog — 喂硬件看门狗（不变）
// ================================================================
void handleWatchdog() { ESP.wdtFeed(); }

// ================================================================
//  setup / loop（不变）
// ================================================================
void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    setStatusLED(false);

    Serial.begin(SERIAL_BAUD);
    while (!Serial) ;
    Serial.setTimeout(10);

    Serial.println("\n[BOOT] ESP-12F PV WiFi Bridge v2.0");
    Serial.printf("[BOOT] dev=%u server=%s:%u %s\n",
        cfg.deviceID, cfg.serverIP.toString().c_str(),
        cfg.serverPort, cfg.useUDP ? "UDP" : "TCP");

    memset(&sensorData, 0, sizeof(sensorData));

    connectWiFi();
    if (cfg.useUDP) udpClient.begin(4399);        // 固定本地端口，操控台回发用
    else tcpClient.setNoDelay(true);

    lastWiFiCheck  = millis();
    lastHeartbeat = millis();
}

void loop() {
    handleWatchdog();               // 喂硬件看门狗，防止系统复位

    processIncomingMCUData();       // 读取 MCU 发来的二进制帧（0x11面板状态/0x14应答等）

    if (Serial.available()) {
        uint8_t firstByte = Serial.peek();
        if (firstByte != WIFI_FRAME_SYNC) readSerialCommands();  // 处理串口文本配置命令
    }

    checkWiFi();                    // WiFi 断线检测与自动重连
    receiveFromServer();            // 接收服务器指令（JSON + 二进制快速指令）
    processOutgoingCommands();      // 将待发命令打包成二进制帧发给 MCU
    periodicStatusReport();         // 状态变化或15秒心跳 → 上报服务器

    if (sensorData.online &&
        (millis() - sensorData.lastUpdate > STATUS_TIMEOUT_MS)) {
        sensorData.online = false;   // MCU 10秒无数据，标记离线
        Serial.println("[WARN] MCU timeout (10s)");
    }

    delay(10);                      // 让出 CPU 给 WiFi 协议栈处理网络包
}
