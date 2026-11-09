#pragma once
// Mau cau hinh. Copy thanh config.h (da bi .gitignore bo qua) roi dien thong tin that.

// ---- Wi-Fi (chi 2.4 GHz) ----
#define WIFI_SSID "CHANGE_ME"
#define WIFI_PASS "CHANGE_ME"

// ---- MQTT qua TLS ----
#define MQTT_HOST "broker.emqx.io"
#define MQTT_PORT 8883
#define MQTT_USER ""             // de trong neu broker khong can tai khoan
#define MQTT_PASS ""

#define DEVICE_ID "BAS_LIDAR_001"
#define MQTT_TOPIC "bas/" DEVICE_ID "/telemetry"
#define MQTT_CLIENT_ID "bas-" DEVICE_ID "-wifi"
#define PUBLISH_INTERVAL_MS 1000UL
#define TZ_OFFSET_S (7 * 3600)
