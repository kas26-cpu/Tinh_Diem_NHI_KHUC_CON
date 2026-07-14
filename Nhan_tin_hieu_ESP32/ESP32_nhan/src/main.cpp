/*
  ============================================================
  ESP32 DevKit V1 - MASTER RECEIVER
  Nhan du lieu ESP-NOW tu 2 node cam bien (RED + BLUE)
  In ra Serial Plotter de theo doi real-time
  ============================================================
  Phan cung : ESP32 DevKit V1 (DOIT 30-pin)
  Core      : Arduino ESP32 2.x (framework-arduinoespressif32 3.20017.x)
  Baud rate : 115200

  Cach hoat dong:
    1. Nhan goi tin ESP-NOW tu node RED (44:BD:8D:27:92:98)
       va node BLUE (9C:CC:01:D1:8A:A8)
    2. Kiem tra magic, version, CRC32
    3. Phan biet RED / BLUE qua truong team_id trong goi tin
    4. In ra Serial Plotter 6 duong:
         RED_jerk, RED_gyro, RED_hit, BLUE_jerk, BLUE_gyro, BLUE_hit

  FORMAT SERIAL PLOTTER (moi dong):
    RED_jerk:x.xx,RED_gyro:x.x,RED_hit:x,BLUE_jerk:x.xx,BLUE_gyro:x.x,BLUE_hit:x
  ============================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ============================================================
// CAU HINH - chinh sua o day neu can
// ============================================================

// Phai giong ESPNOW_CHANNEL ben firmware sensor (ESP32-C3).
static const uint8_t  ESPNOW_CHANNEL   = 1;

// Baud rate Serial Plotter / Monitor.
static const uint32_t SERIAL_BAUD      = 115200;

// In thong ke loi moi bao nhieu ms (dat 0 de tat thong ke).
static const uint32_t STATS_PERIOD_MS  = 5000;

// Neu khong nhan duoc goi tin trong khoang nay -> in 0 (node mat ket noi).
static const uint32_t NODE_TIMEOUT_MS  = 500;

// ============================================================
// GIAO THUC - PHAI GIONG HET BEN FIRMWARE SENSOR
// ============================================================

static const uint16_t PROTOCOL_MAGIC        = 0x5343;
static const uint8_t  PROTOCOL_VERSION      = 1;
static const uint8_t  PROTOCOL_PAYLOAD_SIZE = 32;

// Team ID
#define TEAM_NONE 0
#define TEAM_RED  1
#define TEAM_BLUE 2

// Device type
#define DEVICE_SENSOR 1

// Message type
#define MESSAGE_SENSOR_TELEMETRY 1

// Flags
#define FLAG_NONE              0
#define FLAG_SENSOR_READ_ERROR (1 << 0)

#pragma pack(push, 1)

struct SensorPayload {
  float   gyro_total;      // Van toc quay tong (do/s)
  float   jerk;            // Do giat (g/s)
  uint8_t hit_detected;    // 1 = dang trong cua so 500ms sau cu danh
  uint8_t reserved[23];
};

struct RefereePayload {
  uint8_t controller_id;
  uint8_t action;
  uint8_t pressed;
  uint8_t reserved[29];
};

union PacketPayload {
  SensorPayload  sensor;
  RefereePayload referee;
  uint8_t        raw[32];
};

struct DataPacket {
  uint16_t      magic;
  uint8_t       protocol_version;
  uint8_t       packet_size;
  uint8_t       device_type;
  uint8_t       team_id;
  uint8_t       message_type;
  uint8_t       flags;
  uint32_t      sequence;
  uint32_t      timestamp;
  PacketPayload payload;
  uint32_t      crc32;
};

#pragma pack(pop)

// Kiem tra kich thuoc struct tai compile-time.
static_assert(sizeof(SensorPayload)  == 32, "SensorPayload must be 32 bytes");
static_assert(sizeof(RefereePayload) == 32, "RefereePayload must be 32 bytes");
static_assert(sizeof(DataPacket)     == 52, "DataPacket must be 52 bytes");

// ============================================================
// TINH CRC32 (thuat toan chuan, phai giong ben sender)
// ============================================================

static uint32_t calcCRC32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
    }
  }
  return ~crc;
}

// ============================================================
// TRANG THAI CUA TUNG NODE
// Dung 2 buffer rieng de tranh mat goi tin khi RED va BLUE
// gui cung mot luc.
// ============================================================

struct NodeState {
  DataPacket packet;         // Goi tin moi nhat
  uint32_t   last_rx_ms;    // Thoi diem nhan lan cuoi (millis)
  bool       has_new;       // Co goi tin chua doc
  bool       ever_received; // Da tung nhan duoc chua
};

static NodeState g_red  = {};
static NodeState g_blue = {};

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// THONG KE LOI
// ============================================================

static volatile uint32_t stat_ok         = 0;
static volatile uint32_t stat_bad_length = 0;
static volatile uint32_t stat_bad_magic  = 0;
static volatile uint32_t stat_bad_crc    = 0;
static volatile uint32_t stat_bad_team   = 0;

static uint32_t last_stats_ms = 0;

// ============================================================
// CALLBACK ESP-NOW
// Chay trong WiFi-task (uu tien cao).
// TUYET DOI KHONG goi Serial.print o day.
// Chi copy du lieu vao buffer, dat co, roi thoat nhanh.
//
// Dung signature cua Core 2.x:
//   void cb(const uint8_t *mac, const uint8_t *data, int len)
// ============================================================

void onESPNowReceive(const uint8_t *mac_addr,
                     const uint8_t *data,
                     int            length)
{
  (void)mac_addr; // Khong dung MAC - phan biet team qua team_id trong packet

  if (data == nullptr) return;

  // Kiem tra kich thuoc
  if (length != (int)sizeof(DataPacket)) {
    stat_bad_length++;
    return;
  }

  const DataPacket *pkt = reinterpret_cast<const DataPacket *>(data);

  // Kiem tra magic va version
  if (pkt->magic != PROTOCOL_MAGIC ||
      pkt->protocol_version != PROTOCOL_VERSION) {
    stat_bad_magic++;
    return;
  }

  // Kiem tra CRC32
  // Chi tinh CRC tren phan du lieu truoc truong crc32
  uint32_t expected = calcCRC32(data, offsetof(DataPacket, crc32));
  if (expected != pkt->crc32) {
    stat_bad_crc++;
    return;
  }

  uint32_t now = millis();

  portENTER_CRITICAL(&g_mux);

  if (pkt->team_id == TEAM_RED) {
    memcpy(&g_red.packet,    pkt, sizeof(DataPacket));
    g_red.last_rx_ms    = now;
    g_red.has_new       = true;
    g_red.ever_received = true;
    stat_ok++;
  }
  else if (pkt->team_id == TEAM_BLUE) {
    memcpy(&g_blue.packet,    pkt, sizeof(DataPacket));
    g_blue.last_rx_ms    = now;
    g_blue.has_new       = true;
    g_blue.ever_received = true;
    stat_ok++;
  }
  else {
    stat_bad_team++;
  }

  portEXIT_CRITICAL(&g_mux);
}

// ============================================================
// KHOI DONG ESP-NOW
// ============================================================

bool startESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // WIFI_PS_NONE: khong tiet kiem dien, do tre thap nhat cho viec nhan.
  esp_wifi_set_ps(WIFI_PS_NONE);

  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    return false;

  if (esp_now_init() != ESP_OK)
    return false;

  if (esp_now_register_recv_cb(onESPNowReceive) != ESP_OK) {
    esp_now_deinit();
    return false;
  }

  return true;
}

// ============================================================
// IN RA SERIAL PLOTTER
//
// Format: label:gia_tri,label:gia_tri,...\n
// Serial Plotter cua Arduino IDE / PlatformIO tu dong ve moi
// label thanh 1 duong thi.
//
// 6 duong hien thi:
//   RED_jerk  - Do giat cua RED (g/s). Dot bien cao = va cham manh.
//   RED_gyro  - Van toc quay cua RED (do/s). Cao = xoay nhanh.
//   RED_hit   - Cu danh cua RED: 0 = binh thuong, 1 = dang trong
//               cua so 500ms sau cu danh.
//   BLUE_jerk - Tuong tu cho BLUE.
//   BLUE_gyro - Tuong tu cho BLUE.
//   BLUE_hit  - Tuong tu cho BLUE.
//
// Neu node mat ket noi (> NODE_TIMEOUT_MS), in 0 thay vi giu
// gia tri cu de Plotter khong ve duong gia.
// ============================================================

void printPlotterLine(const NodeState &red,
                      const NodeState &blue,
                      uint32_t         now)
{
  // RED
  bool  r_ok   = red.ever_received && (now - red.last_rx_ms < NODE_TIMEOUT_MS);
  float r_jerk = r_ok ? red.packet.payload.sensor.jerk       : 0.0f;
  float r_gyro = r_ok ? red.packet.payload.sensor.gyro_total : 0.0f;
  int   r_hit  = r_ok ? (int)red.packet.payload.sensor.hit_detected : 0;

  // BLUE
  bool  b_ok   = blue.ever_received && (now - blue.last_rx_ms < NODE_TIMEOUT_MS);
  float b_jerk = b_ok ? blue.packet.payload.sensor.jerk       : 0.0f;
  float b_gyro = b_ok ? blue.packet.payload.sensor.gyro_total : 0.0f;
  int   b_hit  = b_ok ? (int)blue.packet.payload.sensor.hit_detected : 0;

  // In 1 dong hoan chinh
  Serial.print("RED_jerk:");   Serial.print(r_jerk, 2);
  Serial.print(",RED_gyro:");  Serial.print(r_gyro, 1);
  Serial.print(",RED_hit:");   Serial.print(r_hit);
  Serial.print(",BLUE_jerk:"); Serial.print(b_jerk, 2);
  Serial.print(",BLUE_gyro:"); Serial.print(b_gyro, 1);
  Serial.print(",BLUE_hit:");  Serial.println(b_hit);
}

// ============================================================
// IN THONG KE LOI (dinh ky)
// Dong bat dau bang '#' khong bi ve tren Serial Plotter nhung
// van hien thi trong Serial Monitor.
// ============================================================

void printStats() {
  if (STATS_PERIOD_MS == 0) return;
  Serial.print("# ok=");       Serial.print(stat_ok);
  Serial.print(" bad_len=");   Serial.print(stat_bad_length);
  Serial.print(" bad_magic="); Serial.print(stat_bad_magic);
  Serial.print(" bad_crc=");   Serial.print(stat_bad_crc);
  Serial.print(" bad_team=");  Serial.println(stat_bad_team);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Serial.println();
  Serial.println("# ============================================");
  Serial.println("# ESP32 Master - Serial Plotter ESP-NOW");
  Serial.println("# RED  : 44:BD:8D:27:92:98");
  Serial.println("# BLUE : 9C:CC:01:D1:8A:A8");
  Serial.println("# ============================================");

  while (!startESPNow()) {
    Serial.println("# ESP-NOW init that bai, thu lai...");
    delay(500);
  }

  Serial.print("# MAC cua board nay: ");
  Serial.println(WiFi.macAddress());
  Serial.print("# Kenh ESP-NOW: ");
  Serial.println(ESPNOW_CHANNEL);
  Serial.println("# San sang nhan du lieu...");
  Serial.println();

  last_stats_ms = millis();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  // Lay ban sao nhanh roi giai phong lock
  NodeState red_snap, blue_snap;
  bool      red_new, blue_new;

  portENTER_CRITICAL(&g_mux);
  red_snap  = g_red;
  blue_snap = g_blue;
  red_new   = g_red.has_new;
  blue_new  = g_blue.has_new;
  g_red.has_new  = false;
  g_blue.has_new = false;
  portEXIT_CRITICAL(&g_mux);

  uint32_t now = millis();

  // In ngay khi co packet moi tu bat ky node nao
  if (red_new || blue_new) {
    printPlotterLine(red_snap, blue_snap, now);
  }

  // In thong ke dinh ky
  if (STATS_PERIOD_MS > 0 && (now - last_stats_ms >= STATS_PERIOD_MS)) {
    printStats();
    last_stats_ms = now;
  }

  // Nhuong CPU cho WiFi task / idle task
  delay(1);
}
