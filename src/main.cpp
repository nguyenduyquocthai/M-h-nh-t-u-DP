/*
 * Doc du lieu RPLIDAR A1M8 bang ESP32 (khong can thu vien ngoai)
 *
 * Day noi:
 *   RPLIDAR 1 GND     -> GND ESP32 + GND nguon 5V
 *   RPLIDAR 2 RX      -> GPIO17 ESP32 (TX2)
 *   RPLIDAR 3 TX      -> GPIO16 ESP32 (RX2)
 *   RPLIDAR 4 V5.0    -> +5V ngoai
 *   RPLIDAR 5 GND     -> GND
 *   RPLIDAR 6 MOTOCTL -> GPIO25 ESP32
 *   RPLIDAR 7 VMOTO   -> +5V ngoai
 *
 * Giao thuc: UART 115200 8N1 (A1M8). Du lieu in ra Serial (USB).
 *   PRINT_RAW_POINTS = 0 : moi vong quay in 1 dong tom tat + khoang cach nho nhat moi cung 10 do
 *   PRINT_RAW_POINTS = 1 : in tung diem "angle_deg,distance_mm,quality" (can monitor 921600 baud)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#if __has_include("config.h")
#include "config.h"            // cau hinh rieng (Wi-Fi, MQTT) - khong dua len git
#else
#include "config.example.h"
#endif
#include "ca_cert.h"

// ---------------- Cau hinh ----------------
#define LIDAR_RX_PIN     16      // ESP32 RX2  <- RPLIDAR TX
#define LIDAR_TX_PIN     17      // ESP32 TX2  -> RPLIDAR RX
#define LIDAR_MOTOR_PIN  25      // MOTOCTL (muc cao = quay)
#define LIDAR_BAUD       115200

#define PRINT_RAW_POINTS 1
#define USB_BAUD         (PRINT_RAW_POINTS ? 921600 : 115200)

// ---------------- Lenh RPLIDAR ----------------
#define CMD_SYNC   0xA5
#define CMD_STOP   0x25
#define CMD_RESET  0x40
#define CMD_SCAN   0x20
#define CMD_INFO   0x50
#define CMD_HEALTH 0x52

#define SECTOR_DEG 10
#define NUM_SECTORS (360 / SECTOR_DEG)

HardwareSerial &lidar = Serial2;

// ---------------- Ham tien ich ----------------
static void sendCmd(uint8_t cmd) {
  lidar.write((uint8_t)CMD_SYNC);
  lidar.write(cmd);
}

static void clearRx() {
  while (lidar.available()) lidar.read();
}

static bool waitBytes(size_t n, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while ((size_t)lidar.available() < n) {
    if (millis() - t0 > timeoutMs) return false;
    delay(1);
  }
  return true;
}

// Doc response descriptor 7 byte: A5 5A | len(30bit)+mode(2bit) | type
static bool readDescriptor(uint32_t &len, uint8_t &type, uint32_t timeoutMs = 1000) {
  uint32_t t0 = millis();
  uint8_t d[7];
  uint8_t i = 0;
  while (i < 7) {
    if (millis() - t0 > timeoutMs) return false;
    if (!lidar.available()) { delay(1); continue; }
    uint8_t b = lidar.read();
    if (i == 0 && b != 0xA5) continue;
    if (i == 1 && b != 0x5A) { i = (b == 0xA5) ? 1 : 0; continue; }
    d[i++] = b;
  }
  len  = (uint32_t)d[2] | ((uint32_t)d[3] << 8) | ((uint32_t)d[4] << 16) | ((uint32_t)(d[5] & 0x3F) << 24);
  type = d[6];
  return true;
}

static void motorOn()  { digitalWrite(LIDAR_MOTOR_PIN, HIGH); }
static void motorOff() { digitalWrite(LIDAR_MOTOR_PIN, LOW); }

static bool printDeviceInfo() {
  clearRx();
  sendCmd(CMD_INFO);
  uint32_t len; uint8_t type;
  if (!readDescriptor(len, type) || len != 20 || !waitBytes(20, 500)) return false;
  uint8_t r[20];
  for (int i = 0; i < 20; i++) r[i] = lidar.read();
  Serial.printf("Model: %u, Firmware: %u.%02u, Hardware: %u\n", r[0], r[2], r[1], r[3]);
  Serial.print("Serial: ");
  for (int i = 4; i < 20; i++) Serial.printf("%02X", r[i]);
  Serial.println();
  return true;
}

// 0 = tot, 1 = canh bao, 2 = loi, -1 = khong phan hoi
static int getHealth() {
  clearRx();
  sendCmd(CMD_HEALTH);
  uint32_t len; uint8_t type;
  if (!readDescriptor(len, type) || len != 3 || !waitBytes(3, 500)) return -1;
  uint8_t status = lidar.read();
  uint8_t lo = lidar.read();
  uint8_t hi = lidar.read();
  Serial.printf("Health status=%u error_code=0x%04X\n", status, (uint16_t)(lo | (hi << 8)));
  return status;
}

static bool startScan() {
  clearRx();
  sendCmd(CMD_SCAN);
  uint32_t len; uint8_t type;
  if (!readDescriptor(len, type) || len != 5 || type != 0x81) return false;
  return true;
}

// ---------------- Xu ly du lieu quet ----------------
static uint32_t revCount = 0;
static uint16_t pointsInRev = 0;
static float    nearestDist = 0, nearestAngle = 0;
static uint16_t sectorMin[NUM_SECTORS];
static uint16_t binMm[360];             // khoang cach (mm) theo tung 1 do cua vong dang quet

// Vong quay hoan chinh gan nhat, chia se cho task mang
static SemaphoreHandle_t snapMutex;
static uint16_t snapMm[360];
static uint32_t snapRev = 0, snapPoints = 0, snapTimeMs = 0;
static float    snapNearest = 0, snapNearestAngle = 0;
static bool     snapValid = false;

static void resetRevolution() {
  pointsInRev = 0;
  nearestDist = 0;
  nearestAngle = 0;
  for (int i = 0; i < NUM_SECTORS; i++) sectorMin[i] = 0;
  memset(binMm, 0, sizeof binMm);
}

static void printRevolution() {
  if (pointsInRev == 0) return;
  revCount++;
  if (snapMutex && xSemaphoreTake(snapMutex, 0) == pdTRUE) {   // khong bao gio cho: uu tien doc lidar
    memcpy(snapMm, binMm, sizeof snapMm);
    snapRev = revCount;
    snapPoints = pointsInRev;
    snapNearest = nearestDist;
    snapNearestAngle = nearestAngle;
    snapTimeMs = millis();
    snapValid = true;
    xSemaphoreGive(snapMutex);
  }
  Serial.printf("REV %lu | points=%u | nearest=%.0f mm @ %.1f deg\n",
                (unsigned long)revCount, pointsInRev, nearestDist, nearestAngle);
  Serial.print("  min mm per 10deg: ");
  for (int i = 0; i < NUM_SECTORS; i++) {
    Serial.printf("%d:%u ", i * SECTOR_DEG, sectorMin[i]);
  }
  Serial.println();
}

static void handlePoint(bool startFlag, uint8_t quality, float angle, float dist) {
  if (startFlag) {            // bat dau vong quay moi
    printRevolution();
    resetRevolution();
  }
  if (dist <= 0) return;      // diem khong hop le (khong do duoc)

#if PRINT_RAW_POINTS
  Serial.printf("%.2f,%.1f,%u\n", angle, dist, quality);
#endif

  pointsInRev++;
  if (nearestDist == 0 || dist < nearestDist) {
    nearestDist = dist;
    nearestAngle = angle;
  }
  int s = (int)(angle / SECTOR_DEG) % NUM_SECTORS;
  uint16_t d = (uint16_t)dist;
  if (sectorMin[s] == 0 || d < sectorMin[s]) sectorMin[s] = d;
  int b = (int)angle % 360;
  if (binMm[b] == 0 || d < binMm[b]) binMm[b] = d;
}

// ---------------- Wi-Fi + MQTT (task rieng tren core 0) ----------------
#define WF_LOG(...) do { Serial.print("[WF] "); Serial.printf(__VA_ARGS__); Serial.println(); } while (0)

static char jsonBuf[4096];

// Tra ve do dai JSON, 0 neu khong vua buffer
static size_t buildJson() {
  uint32_t rev = 0, points = 0, age = 0;
  float nearest = 0, nearestAngle = 0;
  bool valid = false;
  static uint16_t mm[360];
  if (xSemaphoreTake(snapMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    valid = snapValid;
    if (valid) {
      memcpy(mm, snapMm, sizeof mm);
      rev = snapRev; points = snapPoints; nearest = snapNearest; nearestAngle = snapNearestAngle;
      age = millis() - snapTimeMs;
    }
    xSemaphoreGive(snapMutex);
  }
  const char *status = !valid ? "no_data" : age > 3000 ? "stale" : "ok";
  int n = snprintf(jsonBuf, sizeof jsonBuf,
    "{\"sensor\":\"RPLIDAR-A1M8\",\"status\":\"%s\",\"unit\":\"mm\",\"step_deg\":1,"
    "\"rev\":%lu,\"points\":%lu,\"nearest_mm\":%.0f,\"nearest_deg\":%.1f,"
    "\"age_ms\":%lu,\"rssi\":%d,\"uptime_s\":%lu",
    status, (unsigned long)rev, (unsigned long)points, nearest, nearestAngle,
    (unsigned long)age, (int)WiFi.RSSI(), (unsigned long)(millis() / 1000));
  if (n < 0 || (size_t)n >= sizeof jsonBuf) return 0;
  size_t len = n;
  if (!strcmp(status, "ok")) {
    len += snprintf(jsonBuf + len, sizeof jsonBuf - len, ",\"ranges\":[");
    for (int i = 0; i < 360; i++) {
      if (sizeof jsonBuf - len < 16) return 0;
      if (mm[i]) len += snprintf(jsonBuf + len, sizeof jsonBuf - len, "%u%s", mm[i], i < 359 ? "," : "");
      else       len += snprintf(jsonBuf + len, sizeof jsonBuf - len, "null%s", i < 359 ? "," : "");
    }
    len += snprintf(jsonBuf + len, sizeof jsonBuf - len, "]");
  }
  if (sizeof jsonBuf - len < 3) return 0;
  jsonBuf[len++] = '}';
  jsonBuf[len] = 0;
  return len;
}

static bool wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WF_LOG("Dang ket noi Wi-Fi \"%s\"...", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(250);
  if (WiFi.status() != WL_CONNECTED) {
    WF_LOG("Khong ket noi duoc Wi-Fi (status %d): kiem tra ten/mat khau, mang phai la 2.4 GHz", (int)WiFi.status());
    WiFi.disconnect();
    return false;
  }
  WF_LOG("IP %s, RSSI %d dBm, kenh %d", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(), (int)WiFi.channel());
  return true;
}

static bool timeSync() {
  if (time(nullptr) > 1735689600) return true;      // da dong bo (sau 2025-01-01)
  WF_LOG("Dong bo gio NTP...");
  configTime(TZ_OFFSET_S, 0, "pool.ntp.org", "time.google.com");
  uint32_t t0 = millis();
  while (time(nullptr) < 1735689600 && millis() - t0 < 20000) delay(250);
  if (time(nullptr) < 1735689600) { WF_LOG("Khong dong bo duoc gio (UDP 123 bi chan?)"); return false; }
  WF_LOG("Gio OK");
  return true;
}

static void netTask(void *) {
  WiFiClientSecure tls;
  PubSubClient mqtt(tls);
  uint32_t lastPublish = 0, publishCount = 0;
  for (;;) {
    if (!wifiConnect() || !timeSync()) { delay(5000); continue; }
    if (!mqtt.connected()) {
      tls.stop();
      tls.setCACert(CA_CERT_PEM);                    // xac thuc chung chi va hostname cua broker
      tls.setHandshakeTimeout(30);
      mqtt.setServer(MQTT_HOST, MQTT_PORT);
      mqtt.setSocketTimeout(20);
      mqtt.setKeepAlive(60);
      mqtt.setBufferSize(4096);
      WF_LOG("MQTT TLS toi %s:%d (client %s)", MQTT_HOST, MQTT_PORT, MQTT_CLIENT_ID);
      bool ok = mqtt.connect(MQTT_CLIENT_ID, strlen(MQTT_USER) ? MQTT_USER : nullptr,
                             strlen(MQTT_PASS) ? MQTT_PASS : nullptr);
      if (!ok) {
        char err[128] = "";
        int code = tls.lastError(err, sizeof err);
        WF_LOG("MQTT that bai: state %d, TLS %d %s. Thu lai sau 5 s", mqtt.state(), code, err);
        delay(5000);
        continue;
      }
      WF_LOG("MQTT da ket noi, topic %s", MQTT_TOPIC);
      lastPublish = millis() - PUBLISH_INTERVAL_MS;
    }
    mqtt.loop();
    if (millis() - lastPublish >= PUBLISH_INTERVAL_MS) {
      lastPublish = millis();
      size_t len = buildJson();
      if (!len) { WF_LOG("Payload khong vua buffer, bo qua"); }
      else if (mqtt.publish(MQTT_TOPIC, (const uint8_t *)jsonBuf, len, false)) {
        if (++publishCount % 10 == 1) WF_LOG("Publish #%lu OK (%u byte)", (unsigned long)publishCount, (unsigned)len);
      } else {
        WF_LOG("Publish that bai, state %d: ket noi lai", mqtt.state());
        mqtt.disconnect();
      }
    }
    delay(20);
  }
}

// Moi diem 5 byte:
//  b0: quality[7:2] | !S[1] | S[0]
//  b1: angle_q6[6:0] | C[0] (C luon = 1)
//  b2: angle_q6[14:7]
//  b3,b4: distance_q2 (little endian)
static void parseScanStream() {
  static uint8_t buf[5];
  static uint8_t idx = 0;

  while (lidar.available()) {
    uint8_t b = lidar.read();

    if (idx == 0) {
      if ((b & 0x01) == ((b >> 1) & 0x01)) continue;   // S va !S phai khac nhau
    } else if (idx == 1) {
      if (!(b & 0x01)) {                               // bit C phai = 1 -> lech byte
        idx = 0;
        if ((b & 0x01) != ((b >> 1) & 0x01)) buf[idx++] = b;
        continue;
      }
    }

    buf[idx++] = b;
    if (idx < 5) continue;
    idx = 0;

    bool    startFlag = buf[0] & 0x01;
    uint8_t quality   = buf[0] >> 2;
    float   angle     = (((uint16_t)buf[2] << 8) | buf[1]) >> 1;
    angle            /= 64.0f;
    float   dist      = (((uint16_t)buf[4] << 8) | buf[3]) / 4.0f;

    handlePoint(startFlag, quality, angle, dist);
  }
}

// ---------------- Arduino ----------------
static bool initLidar() {
  // RESET de dua lidar ve trang thai sach (dac biet khi ESP32 reset ma lidar van dang quet)
  sendCmd(CMD_RESET);
  delay(2000);
  clearRx();

  if (!printDeviceInfo()) {
    Serial.println("Khong doc duoc INFO - kiem tra day RX/TX va nguon 5V");
    return false;
  }
  int h = getHealth();
  if (h < 0) { Serial.println("Khong doc duoc HEALTH"); return false; }
  if (h == 2) { Serial.println("Lidar bao LOI (protection stop) - thu cap nguon lai"); return false; }
  return true;
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(500);
  Serial.println("\n=== RPLIDAR A1M8 + ESP32 ===");

  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  motorOn();

  lidar.setRxBufferSize(4096);
  lidar.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  delay(1000);                // cho motor dat toc do
  while (!initLidar()) {
    delay(1000);
  }

  resetRevolution();
  while (!startScan()) {
    Serial.println("Start scan that bai, thu lai...");
    delay(1000);
  }
  Serial.println("Dang quet...");

  snapMutex = xSemaphoreCreateMutex();
  // Mang chay tren core 0 de TLS/Wi-Fi khong lam nghen viec doc lidar (loop chay core 1)
  xTaskCreatePinnedToCore(netTask, "net", 16384, nullptr, 1, nullptr, 0);
}

void loop() {
  parseScanStream();
  // Neu mat du lieu qua 2s -> khoi dong lai quet
  static uint32_t lastData = millis();
  static uint32_t lastCount = 0;
  if (pointsInRev != lastCount) { lastCount = pointsInRev; lastData = millis(); }
  if (millis() - lastData > 2000) {
    Serial.println("Mat du lieu, khoi dong lai quet...");
    sendCmd(CMD_STOP);
    delay(10);
    startScan();
    lastData = millis();
  }
}
