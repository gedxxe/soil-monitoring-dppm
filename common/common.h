#pragma once
#include <Arduino.h>

#define NUM_SLAVES 7

// Jenis paket sederhana
enum PacketType : uint8_t {
  PKT_BEACON = 1,   // master -> broadcast
  PKT_DATA   = 2,   // slave  -> master
  PKT_ACK    = 3    // master -> slave (balasan)
};

// Beacon dari master (disiarkan berkala)
typedef struct __attribute__((packed)) {
  uint8_t  type;           // PKT_BEACON
  uint8_t  channel;        // channel AP master
  uint8_t  masterMac[6];   // MAC STA master
  uint32_t masterUptimeMs; // millis() master
} BeaconPacket;

// Data kelembapan dari slave
typedef struct __attribute__((packed)) {
  uint8_t  type;       // PKT_DATA
  uint8_t  slaveId;    // 1..7
  uint16_t raw;        // 0..4095
  float    kalman;
  float    percent;    // 0..100
  uint32_t millis_ts;  // millis() di slave
  uint32_t seq;        // sequence number incrementing
} SoilPacket;

// ACK dari master ke slave
typedef struct __attribute__((packed)) {
  uint8_t  type;     // PKT_ACK
  uint8_t  slaveId;  // echo
  uint32_t seq;      // seq yg di-ACK
} AckPacket;

// Broadcast address utk beacon
static const uint8_t ESPNOW_BROADCAST_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
