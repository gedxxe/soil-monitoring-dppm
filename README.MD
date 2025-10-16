# Soil Moisture Monitoring System (ESP32 + ESP-NOW + WebSocket)

This repository provides a complete modular system for real-time soil moisture monitoring using multiple ESP32 microcontrollers. The architecture is based on a **distributed sensor network** of eight ESP32 boards: seven act as **slaves** (each connected to a soil moisture sensor), and one acts as a **master** (aggregating all sensor data, managing wireless synchronization, and publishing data to clients over WebSocket).

The implementation emphasizes **robust communication**, **auto-channel discovery**, **noise reduction via Kalman filtering**, and **real-time visualization** through a WebSocket interface.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture](#architecture)
3. [Features](#features)
4. [Hardware Requirements](#hardware-requirements)
5. [Software Requirements](#software-requirements)
6. [Repository Structure](#repository-structure)
7. [Communication Protocol](#communication-protocol)
8. [Kalman Filter Implementation](#kalman-filter-implementation)
9. [Setup and Configuration](#setup-and-configuration)
10. [Running the System](#running-the-system)
11. [Data Format and WebSocket Output](#data-format-and-websocket-output)
12. [Troubleshooting](#troubleshooting)
13. [Future Improvements](#future-improvements)
14. [License](#license)

---

## System Overview

This project enables **wireless soil moisture monitoring** using a cluster of ESP32 boards communicating via **ESP-NOW**. Each slave node reads analog data from a soil moisture sensor, filters it using a Kalman filter to reduce noise, and sends the processed data to the master node.
The master node:

* Automatically detects the Wi-Fi channel of a specified SSID (for Wi-Fi + ESP-NOW coexistence)
* Broadcasts beacon packets so that slaves can synchronize automatically
* Receives and acknowledges data packets from all slaves
* Publishes aggregated readings to a WebSocket endpoint for real-time visualization

The system is designed to be **scalable**, **resilient**, and **network-agnostic**, operating reliably even in unstable wireless environments.

---

## Architecture

**Network topology:**

```
                +-------------------+
                |     Master ESP32   |
                |-------------------|
                | Wi-Fi STA (AP: UEESRG)
                | ESP-NOW Broadcast  |
                | WebSocket Server   |
                +---------+----------+
                          ^
                          |
       ---------------------------------------------
       |          |          |          |          |
   +-------+  +-------+  +-------+  +-------+  +-------+
   |Slave 1|  |Slave 2|  |Slave 3|  |Slave 4|  |Slave 5|
   +-------+  +-------+  +-------+  +-------+  +-------+
      |           |           |           |           |
  Soil Sensor  Soil Sensor  Soil Sensor  Soil Sensor  Soil Sensor
```

**Key communication layers:**

1. **ESP-NOW (2.4GHz)**: Lightweight, low-latency communication between slaves and master.
2. **Wi-Fi STA mode**: Master connects to a Wi-Fi network for WebSocket communication.
3. **WebSocket (port 81)**: Real-time data broadcast to clients (e.g., browser dashboards).

---

## Features

* **Fully automatic channel synchronization**
  Slaves scan channels 1–13 until they detect a beacon from the master, automatically configuring to the correct Wi-Fi channel.

* **Kalman-filtered sensor readings**
  Each slave applies a one-dimensional Kalman filter to smooth sensor data and reduce analog noise.

* **ACK and retransmission system**
  Each data packet includes a sequence number. The master sends acknowledgments (ACK) to confirm receipt. Slaves retry transmission if no ACK is received.

* **Beacon-based synchronization**
  The master broadcasts a beacon every second containing channel, MAC address, and uptime information. Slaves can dynamically resync if disconnected.

* **WebSocket data publishing**
  The master hosts a simple HTTP + WebSocket server allowing clients to view live data streams in JSON format.

* **Health metrics**
  Data packets include timestamps, sequence counters, freshness indicators, and loss counters.

* **Modular code structure**
  Common data structures and Kalman filter logic are split into dedicated files for maintainability and extensibility.

---

## Hardware Requirements

* **8 × ESP32 Development Boards** (DOIT ESP32 DEVKIT V1 or similar)
* **7 × Analog Soil Moisture Sensors**
* **Breadboard and jumper wires**
* **Power supply (5V USB or regulated external source)**

---

## Software Requirements

* **Arduino IDE 2.x or PlatformIO**
* **ESP32 Arduino Core ≥ v3.0.0**
* Required libraries:

  * `WiFi.h`
  * `esp_now.h`
  * `WebServer.h`
  * `WebSocketsServer.h`

---

## Repository Structure

```
soil-monitoring/
├── common/
│   └── common.h                  # Shared packet definitions and constants
├── slave_node/
│   ├── slave_node.ino            # Slave ESP32 firmware
│   ├── KalmanFilter.h            # Kalman filter class
│   └── KalmanFilter.cpp
└── master_node/
    └── master_node.ino           # Master ESP32 firmware
```

---

## Communication Protocol

### Packet Types

| Packet Type  | Direction          | Description                                   |
| ------------ | ------------------ | --------------------------------------------- |
| `PKT_BEACON` | Master → Broadcast | Broadcasts channel, master MAC, uptime        |
| `PKT_DATA`   | Slave → Master     | Contains filtered soil moisture readings      |
| `PKT_ACK`    | Master → Slave     | Acknowledges data receipt and sequence number |

### Packet Structures

**Beacon Packet**

```c
typedef struct {
  uint8_t type;           // PKT_BEACON
  uint8_t channel;        // Wi-Fi channel
  uint8_t masterMac[6];   // Master MAC address
  uint32_t masterUptimeMs;
} BeaconPacket;
```

**Data Packet**

```c
typedef struct {
  uint8_t type;       // PKT_DATA
  uint8_t slaveId;
  uint16_t raw;
  float kalman;
  float percent;
  uint32_t millis_ts;
  uint32_t seq;
} SoilPacket;
```

**Acknowledgment Packet**

```c
typedef struct {
  uint8_t type;       // PKT_ACK
  uint8_t slaveId;
  uint32_t seq;
} AckPacket;
```

---

## Kalman Filter Implementation

Each slave node applies a simple 1D Kalman filter to the analog sensor readings:

```cpp
float KalmanFilter::update(float z) {
  if (!inited) { X = z; inited = true; }
  P += Q;
  float K = P / (P + R);
  X = X + K * (z - X);
  P = (1 - K) * P;
  return X;
}
```

**Parameters:**

* `Q`: Process noise (affects smoothness)
* `R`: Measurement noise (affects responsiveness)
* `P`: Estimation uncertainty
* `X`: Estimated state

Tuning `Q` and `R` allows the system to balance responsiveness vs. noise suppression.

---

## Setup and Configuration

### Step 1: Connect Hardware

* Each slave connects its soil moisture sensor output to analog pin **GPIO 34**.
* Ensure all ESP32 boards share a **common ground**.
* Power each board via USB or a regulated 5V supply.

### Step 2: Flash the Master

* Upload `master_node/master_node.ino`.
* Connect to serial monitor at 115200 baud.
* The master will connect to Wi-Fi (`UEESRG` / `dppmueesrg`) and display:

  * Master MAC address
  * Detected Wi-Fi channel
  * Local IP address

### Step 3: Flash the Slaves

* Upload `slave_node/slave_node.ino` to each slave.
* Modify `#define SLAVE_ID` for each (1–7).
* No need to manually set channel or MAC — these are auto-detected from master beacons.

### Step 4: Test Communication

* Power all boards.
* Master will print received data and ACK responses.
* Access the master’s IP address via a browser:

  ```
  http://<MASTER_IP>/
  ```
* A live JSON data stream should appear under WebSocket updates.

---

## Data Format and WebSocket Output

Each WebSocket message is a structured JSON payload containing all slaves’ data and metadata:

```json
{
  "raw": [2048, 1990, 2085, 2100, 2050, 2075, 2020],
  "kal": [2030.52, 1987.21, 2078.44, 2096.03, 2049.12, 2071.77, 2018.55],
  "pct": [45.3, 47.1, 43.8, 42.9, 45.7, 44.2, 46.8],
  "seq": [121, 119, 118, 122, 120, 117, 115],
  "slaveMillis": [40354, 40500, 40423, 40615, 40218, 40380, 40545],
  "lastSeenMs": [60200, 60180, 60210, 60170, 60200, 60230, 60190],
  "ageMs": [1050, 990, 1100, 980, 1020, 1040, 1000],
  "ok": [true, true, true, true, true, true, true],
  "lostCount": [0, 1, 0, 0, 0, 2, 0],
  "meta": {
    "masterIP": "192.168.1.40",
    "masterMac": "24:6F:28:AA:BB:CC",
    "channel": 6,
    "uptimeMs": 120000
  }
}
```

---

## Troubleshooting

| Issue                  | Possible Cause                      | Solution                                                           |
| ---------------------- | ----------------------------------- | ------------------------------------------------------------------ |
| Slaves never connect   | Wrong SSID or Wi-Fi channel         | Verify master successfully connects to Wi-Fi; check router channel |
| No ACK received        | Weak ESP-NOW signal or interference | Reduce distance, use channel 1/6/11 only                           |
| High noise in readings | Sensor calibration or poor ground   | Recalibrate `ADC_DRY` and `ADC_WET`, ensure shared GND             |
| WebSocket not loading  | Wrong IP or network isolation       | Ensure browser device is on same Wi-Fi network as master           |

---

## Future Improvements

* Integration with MQTT or REST endpoints for cloud dashboards
* Optional SD-card data logging
* OTA firmware updates
* RSSI-based health monitoring
* Dynamic node registration with secure pairing
* Integration with greenhouse or irrigation controllers

---

## License

This project is released under the **MIT License**.
You are free to use, modify, and distribute the software, provided attribution is maintained.

---

### Author Notes

This documentation and implementation aim to demonstrate a robust, modular IoT architecture that balances simplicity, extensibility, and reliability. It can serve as a foundation for larger-scale agricultural monitoring or environmental sensing systems, with straightforward pathways to expand into multi-sensor or multi-network topologies.
