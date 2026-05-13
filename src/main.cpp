#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "protocol.h"

// ========== Global state ==========
DeviceConfig  cfg = DEFAULT_CONFIG;
SensorData    sensorData;
CommStats     stats;
WiFiClient    tcpClient;
WiFiUDP       udpClient;

uint32_t lastHeartbeat  = 0;
uint32_t lastWiFiCheck  = 0;
uint32_t lastStatusSent = 0;

bool    pendingCommand = false;
uint8_t pendingAction  = 0;

// ========== Status LED ==========
void setStatusLED(bool connected) {
    digitalWrite(STATUS_LED_PIN, connected ? LOW : HIGH);
}

// ========== JSON serialization for server status report ==========
void buildStatusJSON(char *buf, size_t len) {
    snprintf(buf, len,
        "{\"dev\":%u,\"mode\":%d,\"rssi\":%d,\"ts\":%lu}",
        cfg.deviceID,
        sensorData.stepperMode,
        WiFi.RSSI(),
        millis());
}

// ========== Parse server JSON command ==========
// Expected format: {"cmd":1,"dev":2,"act":1}  OR  {"cmd":1,"dev":2,"act":2}
void parseServerCommand(const char *json) {
    // Extract device ID
    const char *devPos = strstr(json, "\"dev\":");
    if (!devPos) return;
    uint8_t dev = (uint8_t)atoi(devPos + 6);

    if (dev != cfg.deviceID) return;  // Not for us

    // Extract action
    const char *actPos = strstr(json, "\"act\":");
    if (!actPos) return;
    uint8_t act = (uint8_t)atoi(actPos + 6);

    if (act == ACT_CLEAN || act == ACT_RESET) {
        pendingAction = act;
        pendingCommand = true;
        Serial.printf("[CMD] server -> MCU: dev=%u act=%u\n", dev, act);
    }
}

// ========== Serial text command processing ==========
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
            WiFi.isConnected() ? "OK" : "DOWN",
            WiFi.RSSI(),
            cfg.deviceID,
            sensorData.stepperMode,
            cfg.serverIP.toString().c_str(),
            cfg.serverPort,
            cfg.useUDP ? "UDP" : "TCP");
        Serial.printf("[STATS] MCU frames sent:%u recv:%u bad:%u | server sent:%u recv:%u\n",
            stats.framesSent, stats.framesRecv, stats.badFrames,
            stats.serverSent, stats.serverRecv);
        return;
    }
    Serial.println("[CMD] unknown: " + line);
}

// ========== Read serial text commands (line-buffered) ==========
void readSerialCommands() {
    static String cmdBuf = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdBuf.length() > 0) {
                processSerialCommand(cmdBuf);
                cmdBuf = "";
            }
        } else {
            cmdBuf += c;
        }
    }
}

// ========== WiFi connection management ==========
void connectWiFi() {
    if (WiFi.isConnected()) return;
    Serial.printf("[WIFI] connecting to %s ...\n", cfg.ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
}

void checkWiFi() {
    if (millis() - lastWiFiCheck < WIFI_CHECK_INTERVAL) return;
    lastWiFiCheck = millis();

    if (!WiFi.isConnected()) {
        connectWiFi();
        setStatusLED(false);
    } else {
        setStatusLED(true);
    }
}

// ========== Server connection (TCP) ==========
bool ensureServerConnected() {
    if (cfg.useUDP) return true;
    if (tcpClient.connected()) return true;
    if (tcpClient.connect(cfg.serverIP, cfg.serverPort)) {
        Serial.println("[NET] TCP connected");
        return true;
    }
    return false;
}

// ========== Send data to server ==========
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

// ========== Receive data from server ==========
void receiveFromServer() {
    if (cfg.useUDP) {
        // Check UDP for incoming packets
        int pktSize = udpClient.parsePacket();
        if (pktSize > 0) {
            char buf[256];
            int n = udpClient.read(buf, sizeof(buf) - 1);
            if (n > 0) {
                buf[n] = '\0';
                stats.serverRecv++;
                Serial.printf("[DOWN] %s\n", buf);
                parseServerCommand(buf);
            }
        }
    } else {
        if (tcpClient.connected() && tcpClient.available()) {
            String line = tcpClient.readStringUntil('\n');
            if (line.length() > 0) {
                stats.serverRecv++;
                Serial.printf("[DOWN] %s\n", line.c_str());
                parseServerCommand(line.c_str());
            }
        }
    }
}

// ========== Process incoming MCU data from Serial ==========
void processIncomingMCUData() {
    if (Serial.available() == 0) return;

    // Peek first byte to distinguish frame data from text commands
    uint8_t firstByte = Serial.peek();

    // MCU frames start with 0x00 (FRAME_HDR_BYTE0)
    if (firstByte == FRAME_HDR_BYTE0) {
        while (Serial.available()) {
            uint8_t b = Serial.read();
            uint8_t frame[FRAME_LEN];
            if (parseMCUFrame(b, frame)) {
                stats.framesRecv++;
                if (extractSensorData(frame, sensorData, cfg.deviceID)) {
                    // Data updated successfully
                }
            } else if (rxState == RX_IDLE && b != FRAME_HDR_BYTE0) {
                // State machine reset, may be start of text command
                // Unread is not available; if it's text, it will be handled
                // in next loop iteration by readSerialCommands
                stats.badFrames++;
            }
        }
    }
}

// ========== Send pending command to MCU ==========
void processOutgoingCommands() {
    if (!pendingCommand) return;
    if (!sensorData.online) {
        // MCU not responding; queue the command for later or drop
        Serial.println("[WARN] MCU offline, dropping pending command");
        pendingCommand = false;
        return;
    }

    sendCommandToMCU(cfg.deviceID, pendingAction);
    stats.framesSent++;
    Serial.printf("[TXMCU] dev=%u act=%u\n", cfg.deviceID, pendingAction);
    pendingCommand = false;
}

// ========== Periodic status report to server ==========
void periodicStatusReport() {
    if (millis() - lastStatusSent < STATUS_SEND_INTERVAL) return;
    lastStatusSent = millis();

    if (!WiFi.isConnected()) return;

    char json[256];
    buildStatusJSON(json, sizeof(json));

    if (sendToServer(json)) {
        Serial.printf("[SEND] %s\n", json);
    } else {
        Serial.printf("[SEND] FAIL %s\n", json);
    }
}

// ========== Watchdog ==========
void handleWatchdog() {
    ESP.wdtFeed();
}

// ========== Setup ==========
void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    setStatusLED(false);

    Serial.begin(SERIAL_BAUD);
    while (!Serial) ;
    Serial.setTimeout(10);

    Serial.println("\n[BOOT] ESP-12F PV WiFi Bridge v1.0");
    Serial.printf("[BOOT] Device ID: %u, Server: %s:%u (%s)\n",
        cfg.deviceID,
        cfg.serverIP.toString().c_str(),
        cfg.serverPort,
        cfg.useUDP ? "UDP" : "TCP");

    // Initialize sensor data
    memset(&sensorData, 0, sizeof(sensorData));
    sensorData.stepperMode = MODE_IDLE;
    sensorData.online = false;

    connectWiFi();

    if (!cfg.useUDP) {
        tcpClient.setNoDelay(true);
    }

    lastHeartbeat  = millis();
    lastWiFiCheck  = millis();
    lastStatusSent = millis();
}

// ========== Main loop ==========
void loop() {
    handleWatchdog();

    // 1. Process MCU data from Serial (11-byte binary frames)
    processIncomingMCUData();

    // 2. Process Serial text commands (not binary frame data)
    if (Serial.available()) {
        uint8_t firstByte = Serial.peek();
        if (firstByte != FRAME_HDR_BYTE0) {
            readSerialCommands();
        }
    }

    // 3. WiFi health check
    checkWiFi();

    // 4. Receive server commands
    receiveFromServer();

    // 5. Forward pending commands to MCU
    processOutgoingCommands();

    // 6. Periodic status report to server
    periodicStatusReport();

    // 7. MCU timeout detection
    if (sensorData.online &&
        (millis() - sensorData.lastUpdate > STATUS_TIMEOUT_MS)) {
        sensorData.online = false;
        Serial.println("[WARN] MCU timeout (no data for 10s)");
    }

    delay(10);
}
