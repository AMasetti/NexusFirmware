#include "telemetry.h"
#include "../../include/config.h"
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// WebSocketsServer is from the arduinoWebSockets library
static WebSocketsServer* ws_server_ = nullptr;
static Telemetry::ParamCallback g_param_cb = nullptr;

// Extract a quoted string value for a given JSON key into out[out_sz].
// Returns true if found. Handles: "key":"value"
static bool extract_str(const char* buf, const char* key, char* out, size_t out_sz) {
    const char* p = strstr(buf, key);
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p = strchr(p, '"');
    if (!p) return false;
    p++;
    const char* end = strchr(p, '"');
    if (!end) return false;
    size_t len = (size_t)(end - p);
    if (len >= out_sz) len = out_sz - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static void ws_event(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            break;

        case WStype_TEXT:
            if (g_param_cb && payload && length > 0) {
                char buf[256];
                size_t n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
                memcpy(buf, payload, n);
                buf[n] = '\0';

                // Reject commands with missing or wrong token
                char token[128] = {};
                if (!extract_str(buf, "\"token\"", token, sizeof(token)) ||
                    strcmp(token, WS_TOKEN) != 0) {
                    break;
                }

                char cmd[64]   = {};
                char joint[32] = {};
                float value    = 0.0f;

                extract_str(buf, "\"cmd\"",   cmd,   sizeof(cmd));
                extract_str(buf, "\"joint\"", joint, sizeof(joint));

                const char* val_p = strstr(buf, "\"value\"");
                if (val_p) {
                    val_p = strchr(val_p, ':');
                    if (val_p) value = strtof(val_p + 1, nullptr);
                }

                if (cmd[0] != '\0') {
                    g_param_cb(cmd, joint, value);
                }
            }
            break;

        default:
            break;
    }
}

Telemetry::Telemetry() : port_(WS_PORT), param_cb_(nullptr), connected_(false) {}

bool Telemetry::begin(const char* ssid, const char* password, uint16_t port) {
    port_ = port;

    WiFi.begin(ssid, password);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
    }
    if (WiFi.status() != WL_CONNECTED) return false;

    ws_server_ = new WebSocketsServer(port);
    ws_server_->begin();
    ws_server_->onEvent(ws_event);
    connected_ = true;

    if (MDNS.begin(MDNS_HOSTNAME)) {
        MDNS.addService("ws", "tcp", port);
        Serial.printf("[mDNS] %s.local ready\n", MDNS_HOSTNAME);
    }

    Serial.printf("[WIFI] MAC=%s\n", WiFi.macAddress().c_str());
    Serial.printf("[WIFI] IP=%s  ws://%s.local:%d\n",
        WiFi.localIP().toString().c_str(), MDNS_HOSTNAME, port);

    return true;
}

void Telemetry::loop() {
    if (ws_server_) ws_server_->loop();
}

void Telemetry::set_param_callback(ParamCallback cb) {
    param_cb_ = cb;
    g_param_cb = cb;
}

bool Telemetry::is_connected() const {
    return connected_ && WiFi.status() == WL_CONNECTED;
}

void Telemetry::build_json(const RobotState& s, char* buf, size_t sz) {
    // Scale raw int16 → physical units
    constexpr float ACCEL_MS2 = 9.81f / 16384.0f;           // ±2g → m/s²
    constexpr float GYRO_RPS  = (1.0f / 131.0f) * 0.01745329252f; // ±250°/s → rad/s

    snprintf(buf, sz,
        "{"
        "\"t\":%lu,"
        "\"imu\":{"
          "\"pitch\":%.4f,\"roll\":%.4f,\"yaw_rate\":%.4f,"
          "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
          "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f"
        "},"
        "\"joints\":{"
          "\"l_hip_roll\":%.4f,\"l_hip_pitch\":%.4f,\"l_knee\":%.4f,\"l_ankle_roll\":%.4f,"
          "\"r_hip_roll\":%.4f,\"r_hip_pitch\":%.4f,\"r_knee\":%.4f,\"r_ankle_roll\":%.4f"
        "}"
        "}",
        (unsigned long)s.timestamp_ms,
        s.imu.pitch_rad, s.imu.roll_rad, s.imu.yaw_rate_rad_s,
        s.raw.accel_x * ACCEL_MS2, s.raw.accel_y * ACCEL_MS2, s.raw.accel_z * ACCEL_MS2,
        s.raw.gyro_x  * GYRO_RPS,  s.raw.gyro_y  * GYRO_RPS,  s.raw.gyro_z  * GYRO_RPS,
        s.legs.left.hip_roll,   s.legs.left.hip_pitch,
        s.legs.left.knee,       s.legs.left.ankle_roll,
        s.legs.right.hip_roll,  s.legs.right.hip_pitch,
        s.legs.right.knee,      s.legs.right.ankle_roll
    );
}

void Telemetry::send_state(const RobotState& state) {
    if (!ws_server_) return;
    build_json(state, tx_buf_, sizeof(tx_buf_));
    ws_server_->broadcastTXT(tx_buf_);
}
