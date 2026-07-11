#pragma once
#include <Arduino.h>

// Mã đội
#define TEAM_NONE 0
#define TEAM_RED 1
#define TEAM_BLUE 2

// Thông số giao thức
const uint16_t PROTOCOL_MAGIC = 0x5343;
const uint8_t PROTOCOL_VERSION = 1;
const uint8_t PROTOCOL_PAYLOAD_SIZE = 32;

enum DeviceType : uint8_t {
  DEVICE_UNKNOWN = 0,
  DEVICE_SENSOR = 1,
  DEVICE_REFEREE_CONTROLLER = 2
};

enum MessageType : uint8_t {
  MESSAGE_UNKNOWN = 0,
  MESSAGE_SENSOR_TELEMETRY = 1,
  MESSAGE_REFEREE_EVENT = 2,
  MESSAGE_HEARTBEAT = 3,
  MESSAGE_TIME_SYNC = 4
};

enum PacketFlags : uint8_t {
  FLAG_NONE = 0,
  FLAG_SENSOR_READ_ERROR = 1 << 0
};

#pragma pack(push, 1)

struct SensorPayload {
  float gyro_total;
  float jerk;
  uint8_t hit_detected;
  uint8_t reserved[23];
};

struct RefereePayload {
  uint8_t controller_id;
  uint8_t action;
  uint8_t pressed;
  uint8_t reserved[29];
};

union PacketPayload {
  SensorPayload sensor;
  RefereePayload referee;
  uint8_t raw[PROTOCOL_PAYLOAD_SIZE];
};

struct DataPacket {
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