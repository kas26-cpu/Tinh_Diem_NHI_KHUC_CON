/*
  ESP32-C3 SuperMini + MPU6050 + ESP-NOW
  Arduino ESP32 Core 3.3.10 / PlatformIO

  FIRMWARE CHỈ GỬI TÍN HIỆU (SENSOR)
  Chon TEAM_ID = TEAM_RED hoac TEAM_BLUE.

  GIAI THICH CAC THAY DOI GIAM NHIET (v2):
  1. SENSOR_WIFI_PS_MODE doi thanh WIFI_PS_NONE:
     WIFI_PS_MIN_MODEM tren ESP32-C3 doi khi khong hoat dong on dinh voi
     ESP-NOW (Espressif bug tracker ghi nhan) - radio bat/tat khong dung
     luc khien firmware gui lai nhieu lan, paradox lam nong hon. WIFI_PS_NONE
     on dinh hon, ket hop giam Tx power + giam CPU la du.
  2. IDLE_DELAY_US tang tu 200us len 500us:
     FreeRTOS idle task can it nhat ~300us lien tuc de chay WFI instruction
     (Wait For Interrupt - che do tiet kiem dien cua Xtensa). 200us qua ngan,
     CPU khong kip nghi. 500us van danh cho 500Hz sampling (2000us/vong).
  3. Serial.printf LOI bi xoa khoi sendSensorPacket:
     sendSensorPacket() co the bi goi lien tuc khi loi. Serial.printf trong
     hot-path gay tranh chap buffer UART, doi khi khoa CPU task > 1ms.
     Thay bang co loi doc trong debug period (an toan hon).
*/

#include <Arduino.h> // Thu vien bat buoc cho PlatformIO
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <esp_wifi.h>

// ============================================================================
// CAU HINH NGUOI DUNG
// ============================================================================

#define TEAM_NONE 0
#define TEAM_RED 1
#define TEAM_BLUE 2

#ifndef TEAM_ID
#define TEAM_ID TEAM_BLUE
#endif

#ifndef SENSOR_SERIAL_DEBUG
#define SENSOR_SERIAL_DEBUG 1
#endif

// Giam xung nhip CPU va cho phep modem sleep de giam sinh nhiet.
// Neu do tre goi tin tang qua muc chap nhan duoc, doi SENSOR_WIFI_PS_MODE
// thanh WIFI_PS_NONE de quay lai hanh vi cu (nong hon nhung do tre thap nhat).
#ifndef SENSOR_CPU_FREQ_MHZ
#define SENSOR_CPU_FREQ_MHZ 80
#endif

// WIFI_PS_NONE on dinh hon voi ESP-NOW tren ESP32-C3.
// Xem giai thich chi tiet trong comment dau file.
#ifndef SENSOR_WIFI_PS_MODE
#define SENSOR_WIFI_PS_MODE WIFI_PS_NONE
#endif

// Thoi gian nghi giua moi vong loop (micro giay).
// 500us: du cho idle task chay WFI, van dam bao lay mau 500Hz (chu ky 2000us).
#ifndef SENSOR_IDLE_DELAY_US
#define SENSOR_IDLE_DELAY_US 500
#endif

// Sensor va Server bat buoc phai dung cung kenh ESP-NOW.
const uint8_t ESPNOW_CHANNEL = 1;

// Wi-Fi Station MAC cua ESP32 Server.
const uint8_t SERVER_MAC[6] = {0x70, 0x4B, 0xCA, 0x26, 0x3A, 0x50};

// ============================================================================
// GIAO THUC CHUNG - GOI CO DINH 52 BYTE
// ============================================================================

const uint16_t PROTOCOL_MAGIC = 0x5343;
const uint8_t PROTOCOL_VERSION = 1;
const uint8_t PROTOCOL_PAYLOAD_SIZE = 32;

enum DeviceType : uint8_t
{
  DEVICE_UNKNOWN = 0,
  DEVICE_SENSOR = 1,
  DEVICE_REFEREE_CONTROLLER = 2
};

enum MessageType : uint8_t
{
  MESSAGE_UNKNOWN = 0,
  MESSAGE_SENSOR_TELEMETRY = 1,
  MESSAGE_REFEREE_EVENT = 2,
  MESSAGE_HEARTBEAT = 3,
  MESSAGE_TIME_SYNC = 4
};

enum PacketFlags : uint8_t
{
  FLAG_NONE = 0,
  FLAG_SENSOR_READ_ERROR = 1 << 0
};

#pragma pack(push, 1)

struct SensorPayload
{
  float gyro_total;
  float jerk;
  uint8_t hit_detected;
  uint8_t reserved[23];
};

// Du phong cho 4 dieu khien trong tai sau nay.
struct RefereePayload
{
  uint8_t controller_id;
  uint8_t action;
  uint8_t pressed;
  uint8_t reserved[29];
};

union PacketPayload
{
  SensorPayload sensor;
  RefereePayload referee;
  uint8_t raw[PROTOCOL_PAYLOAD_SIZE];
};

struct DataPacket
{
  uint16_t magic;
  uint8_t protocol_version;
  uint8_t packet_size;
  uint8_t device_type;
  uint8_t team_id;
  uint8_t message_type;
  uint8_t flags;
  uint32_t sequence;
  uint32_t timestamp;
  PacketPayload payload;
  uint32_t crc32;
};

#pragma pack(pop)

static_assert(sizeof(SensorPayload) == 32, "SensorPayload must be 32 bytes");
static_assert(sizeof(RefereePayload) == 32, "RefereePayload must be 32 bytes");
static_assert(sizeof(DataPacket) == 52, "DataPacket must be 52 bytes");

uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 1)
      {
        crc = (crc >> 1) ^ 0xEDB88320;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return ~crc;
}

bool timeReached(uint32_t now, uint32_t deadline)
{
  return (int32_t)(now - deadline) >= 0;
}

void advanceDeadline(uint32_t now, uint32_t period, uint32_t &deadline)
{
  deadline += period;
  // Neu bi tre qua mot chu ky, bo qua lan cu thay vi chay bu don dap.
  if (timeReached(now, deadline))
  {
    deadline = now + period;
  }
}

// ============================================================================
// FIRMWARE SENSOR
// ============================================================================

static_assert(TEAM_ID == TEAM_RED || TEAM_ID == TEAM_BLUE,
              "TEAM_ID must be TEAM_RED or TEAM_BLUE for a sensor");

const uint8_t SDA_PIN = 5;
const uint8_t SCL_PIN = 6;
const uint8_t MPU_ADDR = 0x68;

const uint32_t SAMPLE_PERIOD_US = 2000; // 500 Hz
const uint32_t SEND_PERIOD_US = 20000;  // 50 Hz
const uint32_t DEBUG_PERIOD_MS = 50;    // Giong toc do hien thi code goc
const uint32_t HIT_HOLD_MS = 500;

const float JERK_THRESHOLD = 3.0;
const float GYRO_THRESHOLD = 150.0;

// Cac bien cua thuat toan goc.
float previous_g_total = 1.0;
uint32_t hit_timer = 0;

// Dinh trong cua so 20 ms de gui ESP-NOW 50 Hz.
float tx_max_jerk = 0.0;
float tx_max_gyro = 0.0;

// Dinh rieng trong cua so 50 ms de debug nhu code goc.
float debug_max_jerk = 0.0;
float debug_max_gyro = 0.0;

uint32_t next_sample_us = 0;
uint32_t next_send_us = 0;
uint32_t last_debug_ms = 0;
uint32_t sequence_number = 0;

uint32_t send_error_count = 0;
uint32_t i2c_error_count = 0;
bool sensor_read_error_in_window = false;
bool last_send_had_error = false; // Co loi gui packet lan cuoi (in trong debug period)

bool writeMPURegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool configureMPU6050()
{
  // Giu nguyen ba lenh cau hinh tu chuong trinh goc.
  if (!writeMPURegister(0x6B, 0x00))
    return false; // Wake up
  if (!writeMPURegister(0x1C, 0x18))
    return false; // Accelerometer +/-16 g
  if (!writeMPURegister(0x1B, 0x08))
    return false; // Gyroscope +/-500 deg/s
  return true;
}

bool readMPU6050(float &jerk, float &gyro_total)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  size_t received = Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);

  if (received != 14 || Wire.available() < 14)
  {
    while (Wire.available())
      Wire.read();
    return false;
  }

  int16_t AcX = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t AcY = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t AcZ = (int16_t)((Wire.read() << 8) | Wire.read());

  float ax_g = AcX / 2048.0;
  float ay_g = AcY / 2048.0;
  float az_g = AcZ / 2048.0;

  float g_total = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  jerk = abs(g_total - previous_g_total);
  previous_g_total = g_total;

  Wire.read();
  Wire.read();
  // Bo qua nhiet do

  int16_t GyX = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t GyY = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t GyZ = (int16_t)((Wire.read() << 8) | Wire.read());

  float gx_deg = GyX / 65.5;
  float gy_deg = GyY / 65.5;
  float gz_deg = GyZ / 65.5;

  gyro_total = sqrt(gx_deg * gx_deg +
                    gy_deg * gy_deg +
                    gz_deg * gz_deg);

  return true;
}

bool startESPNowSensor()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Giam cong suat phat song ESP-NOW xuong muc 8.5dBm de triet tieu viec sinh nhiet
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  // WIFI_PS_NONE giu radio "thuc" 100% thoi gian -> sinh nhiet lien tuc.
  // WIFI_PS_MIN_MODEM cho phep modem sleep giua cac lan gui (chu ky gui la
  // 20ms/50Hz nen co du thoi gian ranh), giam dong tieu thu ma van giu do
  // tre o muc chap nhan duoc cho ESP-NOW.
  esp_wifi_set_ps(SENSOR_WIFI_PS_MODE);

  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK)
  {
    return false;
  }

  if (esp_now_init() != ESP_OK)
  {
    return false;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, SERVER_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK)
  {
    esp_now_deinit();
    return false;
  }

  return true;
}

void sampleSensor()
{
  float jerk = 0.0;
  float gyro_total = 0.0;

  if (!readMPU6050(jerk, gyro_total))
  {
    sensor_read_error_in_window = true;
    i2c_error_count++;
    return;
  }

  // Peak tracking cho goi ESP-NOW 20 ms.
  if (jerk > tx_max_jerk)
    tx_max_jerk = jerk;
  if (gyro_total > tx_max_gyro)
    tx_max_gyro = gyro_total;

  // Peak tracking 50 ms phuc vu Serial Monitor/Plotter.
  if (jerk > debug_max_jerk)
    debug_max_jerk = jerk;
  if (gyro_total > debug_max_gyro)
    debug_max_gyro = gyro_total;

  // Thuat toan phat hien va cham duoc giu nguyen.
  if (jerk > JERK_THRESHOLD && gyro_total < GYRO_THRESHOLD)
  {
    hit_timer = millis();
  }
}

void sendSensorPacket()
{
  DataPacket packet = {};

  packet.magic = PROTOCOL_MAGIC;
  packet.protocol_version = PROTOCOL_VERSION;
  packet.packet_size = sizeof(DataPacket);
  packet.device_type = DEVICE_SENSOR;
  packet.team_id = TEAM_ID;
  packet.message_type = MESSAGE_SENSOR_TELEMETRY;
  packet.flags = sensor_read_error_in_window
                     ? FLAG_SENSOR_READ_ERROR
                     : FLAG_NONE;

  packet.sequence = sequence_number++;
  packet.timestamp = millis();
  packet.payload.sensor.gyro_total = tx_max_gyro;
  packet.payload.sensor.jerk = tx_max_jerk;

  packet.payload.sensor.hit_detected =
      (millis() - hit_timer < HIT_HOLD_MS) ? 1 : 0;

  packet.crc32 = calculateCRC32((uint8_t *)&packet, offsetof(DataPacket, crc32));

  esp_err_t result = esp_now_send(SERVER_MAC, (uint8_t *)&packet, sizeof(packet));

  if (result != ESP_OK)
  {
    send_error_count++;
    // KHONG goi Serial.printf o day vi sendSensorPacket chay trong hot-path
    // (50Hz). Thay vao do dat co va in trong debug period (an toan hon).
    last_send_had_error = true;
  }

  tx_max_jerk = 0.0;
  tx_max_gyro = 0.0;
  sensor_read_error_in_window = false;
}

void printSensorDebug()
{
#if SENSOR_SERIAL_DEBUG
  if (millis() - last_debug_ms < DEBUG_PERIOD_MS)
    return;

  Serial.print("Max_Do_Giat:");
  Serial.print(debug_max_jerk);
  Serial.print(",Max_Toc_Do_Xoay(/10):");
  Serial.print(debug_max_gyro / 10.0);
  Serial.print(",Hit:");
  Serial.print((millis() - hit_timer < HIT_HOLD_MS) ? 5.0 : 0.0);
  Serial.print(",ESP_Send_Errors:");
  Serial.print(send_error_count);
  Serial.print(",I2C_Errors:");
  Serial.println(i2c_error_count);

  // In loi gui neu co (an toan vi ta dang o ngoai hot-path)
  if (last_send_had_error)
  {
    Serial.println("[WARN] Co loi khi gui ESP-NOW (kiem tra ket noi/kenh WiFi)");
    last_send_had_error = false;
  }

  debug_max_jerk = 0.0;
  debug_max_gyro = 0.0;

  last_debug_ms = millis();
#endif
}

void setupSensor()
{
  // Ha xung nhip CPU (mac dinh 160MHz) xuong muc du dung cho workload nay
  // (doc I2C 500Hz + gui ESP-NOW 50Hz) de giam dong tieu thu va sinh nhiet.
  setCpuFrequencyMhz(SENSOR_CPU_FREQ_MHZ);

  Serial.begin(115200);
  delay(200);

#if SENSOR_SERIAL_DEBUG
  Serial.println();
  Serial.println("=== ESP32-C3 MPU6050 SENSOR ===");
  Serial.print("Team: ");
  Serial.println(TEAM_ID == TEAM_RED ? "RED" : "BLUE");
  Serial.print("ESP-NOW channel: ");
  Serial.println(ESPNOW_CHANNEL);
#endif

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(5);

  while (!configureMPU6050())
  {
#if SENSOR_SERIAL_DEBUG
    Serial.println("MPU6050 not found. Retrying...");
#endif
    delay(500);
  }

  // Giu nguyen thoi gian cho 1 giay cua code goc.
  delay(1000);

  while (!startESPNowSensor())
  {
#if SENSOR_SERIAL_DEBUG
    Serial.println("ESP-NOW init failed. Retrying...");
#endif
    delay(500);
  }

#if SENSOR_SERIAL_DEBUG
  Serial.print("Sensor STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("MPU6050 and ESP-NOW ready.");
#endif

  uint32_t now = micros();

  next_sample_us = now;
  next_send_us = now + SEND_PERIOD_US;
  last_debug_ms = millis();
}

void loopSensor()
{
  uint32_t now = micros();

  if (timeReached(now, next_sample_us))
  {
    sampleSensor();
    advanceDeadline(now, SAMPLE_PERIOD_US, next_sample_us);
  }

  now = micros();

  if (timeReached(now, next_send_us))
  {
    sendSensorPacket();
    advanceDeadline(now, SEND_PERIOD_US, next_send_us);
  }

  printSensorDebug();

  // QUAN TRONG: Tang IDLE_DELAY len 500us (tu 200us).
  // FreeRTOS idle task can >= ~300us de thuc thi WFI (Wait For Interrupt),
  // che do tiet kiem nang luong cua nhan Xtensa. Voi 200us chip khong kip
  // nghi. 500us van dam bao 500Hz sampling (chu ky 2000us, con lai 1500us
  // duoc dung cho I2C read + xu ly).
  delayMicroseconds(SENSOR_IDLE_DELAY_US);
}

// ============================================================================
// ARDUINO ENTRY POINTS
// ============================================================================

void setup()
{
  setupSensor();
}

void loop()
{
  loopSensor();
}