<div align="center">

# 🚁 DoorDrone

### ESP32-S3 Quadcopter Flight Controller

*Cascade PID · Blynk MQTT Telemetry · UDP Setpoint · MPU6050 IMU*

---

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5.1-blue?style=flat-square&logo=espressif)](https://github.com/espressif/esp-idf)
[![Target](https://img.shields.io/badge/Target-ESP32--S3-red?style=flat-square&logo=espressif)](https://www.espressif.com/en/products/socs/esp32-s3)
[![Firmware](https://img.shields.io/badge/Firmware-v1.5.0-green?style=flat-square)](https://github.com/Marsovac/DoorDrone-project)
[![Phase](https://img.shields.io/badge/Phase-3%20Cascade%20PID-orange?style=flat-square)](#development-phases)
[![License](https://img.shields.io/badge/License-MIT-lightgrey?style=flat-square)](LICENSE)

</div>

---

## Overview

DoorDrone is a full-stack quadcopter flight controller built on the **ESP32-S3**, developed incrementally through clearly defined engineering phases. It implements a **cascade PID attitude controller**, **MPU6050 IMU fusion** via a complementary filter, **real-time UDP setpoint injection**, and **Blynk MQTT telemetry** — all within the ESP-IDF 5.x framework.

The architecture is designed around a clean separation of concerns: flight-critical tasks are isolated on Core 1, all networking and telemetry runs on Core 0, and every module communicates through mutex-protected shared state with explicit timeout policies.

---

## Table of Contents

- [Development Phases](#development-phases)
- [Hardware](#hardware)
- [Architecture](#architecture)
- [Module Overview](#module-overview)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [Blynk Virtual Pin Map](#blynk-virtual-pin-map)
- [UDP Setpoint Protocol](#udp-setpoint-protocol)
- [PID Tuning Guide](#pid-tuning-guide)
- [Cascade PID Logic](#cascade-pid-logic)
- [Safety System](#safety-system)
- [Project Roadmap](#project-roadmap)
- [License](#license)

---

## Development Phases

| Phase | Status | Scope |
|-------|--------|-------|
| **Phase 1** | ✅ Complete | WiFi, Blynk MQTT, MCPWM ESC PWM, LED, NVS |
| **Phase 2** | ✅ Complete | MPU6050 IMU, complementary filter, boot calibration, telemetry |
| **Phase 3** | ✅ Active | UDP setpoint receiver, cascade PID, full 100 Hz flight loop |
| **Phase 4** | 🔲 Planned | Barometer (altitude hold), battery voltage monitoring |
| **Phase 5** | 🔲 Planned | SIM7670G GNSS / 4G cellular, Traccar GPS tracking |

---

## Hardware

| Component | Detail |
|-----------|--------|
| **MCU** | ESP32-S3-SIM7670G-4G (Waveshare) — Xtensa LX7 dual-core, 160 MHz |
| **IMU** | MPU6050 on HW-123 module — I2C 0x68 (AD0 = GND) |
| **ESCs** | Standard brushless ESC × 4 — 50 Hz PWM, 1000–2000 µs |
| **Frame** | Quad-X configuration |
| **Flash** | 2 MB |
| **Power** | 3S/4S LiPo via PDB |

### Pin Assignment

| Signal | GPIO | Notes |
|--------|------|-------|
| IMU SDA | GPIO 4 | 4.7 kΩ pull-up on HW-123 |
| IMU SCL | GPIO 6 | 4.7 kΩ pull-up on HW-123 |
| ESC M1 (FR CW) | GPIO 1 | MCPWM Group 0 |
| ESC M2 (FL CCW) | GPIO 5 | MCPWM Group 0 |
| ESC M3 (RR CCW) | GPIO 10 | MCPWM Group 1 |
| ESC M4 (RL CW) | GPIO 11 | MCPWM Group 1 |
| Status LED | GPIO 8 | Active high |

### Motor Layout (Quad-X, top view)

```
    M2 (FL CCW) ──── M1 (FR CW)
         |   [FRONT]    |
         |    ── X ──   |
         |              |
    M4 (RL CW)  ──── M3 (RR CCW)
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32-S3                             │
│                                                             │
│   CORE 0                        CORE 1                      │
│   ──────────────────────        ──────────────────────      │
│   WiFi Driver (pinned)          IMU Task (100 Hz, prio 10)  │
│   MQTT/Blynk Client             └─ MPU6050 read             │
│   UDP Setpoint Listener         └─ Complementary filter     │
│   (prio 4)                      └─ Mutex-protected output   │
│   Blynk IMU Publish             │                           │
│   (prio 3, 2 Hz)                Flight Control Task         │
│   app_main monitor              (100 Hz, prio 9)            │
│                                 └─ Read IMU snapshot        │
│                                 └─ Read setpoint snapshot   │
│                                 └─ Outer angle P loop       │
│                                 └─ Inner rate PID loop      │
│                                 └─ Quad-X motor mixer       │
│                                 └─ Write MCPWM comparators  │
└─────────────────────────────────────────────────────────────┘
```

**Why this split?** `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0=y` means the WiFi interrupt load never reaches Core 1. The IMU and flight control tasks get deterministic scheduling, which is required for accurate PID integration timing.

---

## Module Overview

```
main/
├── main.c              # Orchestration: WiFi, MQTT, NVS, Blynk handlers, monitor loop
├── imu.c / imu.h       # MPU6050 driver, complementary filter, 100 Hz task
├── pid.c / pid.h       # Generic PID (derivative-on-measurement, anti-windup)
├── setpoint.c/.h       # UDP listener, 12-byte packet parser, shared setpoint state
├── flight_control.c/.h # MCPWM, cascade PID, Quad-X mixer, arm/disarm, safety
├── setpoint.py         # Python test tool — sends UDP setpoint packets from PC
└── CMakeLists.txt      # ESP-IDF component registration
```

---

## Getting Started

### Prerequisites

- [ESP-IDF v5.5.1](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/get-started/index.html)
- ESP32-S3 target configured (`IDF_TARGET=esp32s3`)
- A Blynk account with a device created on the **MQTT** plan
- Python 3 (stdlib only) for `setpoint.py`

### Build and Flash

```bash
# Clone the repository
git clone https://github.com/Marsovac/DoorDrone-project.git
cd DoorDrone-project

# Set the target
idf.py set-target esp32s3

# Configure credentials (see Configuration section below)
idf.py menuconfig

# Build
idf.py build

# Flash and monitor
idf.py -p COM5 flash monitor
```

### Expected Boot Output

```
[MAIN] ══════════════════════════════════════════
[MAIN]   ESP32-S3 Drone Controller  v1.5.0
[MAIN]   Phase 3: Cascade PID active
[MAIN] ══════════════════════════════════════════
[IMU] MPU6050 found at 0x68 — WHO_AM_I=0x68
[IMU] ╔══════════════════════════════════════════╗
[IMU] ║  BOOT CALIBRATION — DO NOT MOVE DRONE   ║
[IMU] ╚══════════════════════════════════════════╝
[IMU] Calibration done (300/300 samples used)
[WiFi] IP: 192.168.x.x
[SETPOINT] Socket bound — waiting for packets on 0.0.0.0:4210
[MQTT] Connected
```

> **Important:** Keep the drone flat and stationary for approximately 3 seconds during boot calibration. Moving the drone during this window biases the gyro offsets and causes attitude drift at hover.

---

## Configuration

Edit the following constants directly in `main/main.c` before building:

```c
#define WIFI_SSID           "your_ssid"
#define WIFI_PASS           "your_password"
#define BLYNK_TEMPLATE_ID   "TMPLxxxxxxx"
#define BLYNK_TEMPLATE_NAME "your_template_name"
#define BLYNK_AUTH_TOKEN    "your_auth_token"
```

### Key sdkconfig Values

These are pre-configured and should not be changed without understanding the implications:

| Key | Value | Why |
|-----|-------|-----|
| `CONFIG_FREERTOS_HZ` | `100` | 1 tick = 10 ms. `pdMS_TO_TICKS(10)` = 1 tick. Values below 10 ms collapse to 0. |
| `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ` | `160` | Sufficient for 100 Hz PID + IMU. Saves power vs 240 MHz. |
| `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0` | `y` | Keeps WiFi ISR load off Core 1. |
| `CONFIG_ESPTOOLPY_FLASHSIZE` | `2MB` | Must match physical flash on your module. |

---

## Blynk Virtual Pin Map

| Virtual Pin | Direction | Function |
|-------------|-----------|----------|
| **V0** | Downlink | ARM (1) / DISARM (0). Arm refused if IMU unhealthy. |
| **V2** | Uplink | Roll angle (°) — published at 2 Hz |
| **V3** | Uplink | Pitch angle (°) — published at 2 Hz |
| **V4** | Uplink | Yaw angle (°) — published at 2 Hz |
| **V5** | Downlink | Outer angle Kp (slider 0.0–15.0). Sets both roll and pitch Kp simultaneously. |
| **V6** | Downlink | Status LED on/off |
| **V7** | Downlink | ESC calibration trigger (value=1). Must be disarmed. Blocking ~10 s. |
| **V8** | Downlink | Motor test select (1–4) |
| **V9** | Downlink | Motor test pulse width (µs, 1000–2000). Spins the selected motor only. |

---

## UDP Setpoint Protocol

The drone receives flight commands over UDP on port **4210**. This is the primary real-time control channel — it replaces slider-based Blynk control for actual flight.

### Packet Format (12 bytes, big-endian)

| Bytes | Field | Type | Range | Units |
|-------|-------|------|-------|-------|
| 0 | Magic 0 | `uint8` | `0xAB` | — |
| 1 | Magic 1 | `uint8` | `0xCD` | — |
| 2–3 | Throttle | `uint16` | 1000–2000 | µs |
| 4–5 | Roll | `int16` | −450 to +450 | value × 10 → ±45.0° |
| 6–7 | Pitch | `int16` | −450 to +450 | value × 10 → ±45.0° |
| 8–9 | Yaw rate | `int16` | −3000 to +3000 | value × 10 → ±300.0°/s |
| 10 | Flags | `uint8` | reserved | — |
| 11 | Checksum | `uint8` | XOR of bytes 0–10 | — |

### Python Test Tool

```bash
# Send idle heartbeat (keep drone from disarming during bench testing)
python3 main/setpoint.py 192.168.1.105

# Command 15° right roll at 1300 µs throttle
python3 main/setpoint.py 192.168.1.105 --thr 1300 --roll 15

# Stress test at 100 Hz for 10 seconds
python3 main/setpoint.py 192.168.1.105 --hz 100 --time 10
```

If UDP packets stop arriving for more than **600 ms**, the flight controller automatically disarms and drives all motors to minimum.

---

## PID Tuning Guide

### Cascade Architecture

```
Stick angle (°)
      │
  ┌───▼─────────────────────────────────┐
  │  OUTER LOOP — Angle P controller    │  Kp only, no Ki or Kd
  │  input:  stick angle (°)            │
  │  sensor: CF-fused roll/pitch (°)    │
  │  output: target rate (°/s)          │  clamped ±200 °/s
  └───┬─────────────────────────────────┘
      │
  ┌───▼─────────────────────────────────┐
  │  INNER LOOP — Rate PID              │  Full Kp, Ki, Kd
  │  input:  target rate (°/s)          │
  │  sensor: raw gyro (°/s)             │
  │  output: torque correction (µs)     │
  └───┬─────────────────────────────────┘
      │
   Quad-X Motor Mixer → M1–M4 (µs)
```

> **Why is the outer loop P-only?** The outer plant (angle) is the integral of the inner plant (rate). Adding an integrator to the outer loop creates a double integrator in the open-loop transfer function, which produces marginal stability and low-frequency oscillation. The inner loop's Ki handles steady-state disturbances.

### Current Gains (Placeholder — Requires Tethered Tuning)

```c
// flight_control.h

// Outer angle loop
#define FC_ANGLE_ROLL_KP    4.5f
#define FC_ANGLE_PITCH_KP   4.5f

// Inner rate loop
#define FC_ROLL_KP   1.0f    // Start here
#define FC_ROLL_KI   0.0f
#define FC_ROLL_KD   0.0f

#define FC_PITCH_KP  1.0f
#define FC_PITCH_KI  0.0f
#define FC_PITCH_KD  0.0f

#define FC_YAW_KP    2.0f
#define FC_YAW_KI    0.0f
#define FC_YAW_KD    0.0f
```

### Tuning Order

1. **Inner Kp first** — tether the drone, increase `FC_ROLL_KP` until it resists disturbances firmly, then reduce by ~20%.
2. **Inner Kd** — increase until Kp-induced oscillation is damped. Stop before noise amplification becomes audible.
3. **Inner Ki** — only if steady-state roll/pitch offset persists at hover with Kp and Kd set.
4. **Outer Kp via V5 slider** — start at 3.0, increase until attitude response is crisp without overshoot.

---

## Safety System

The flight controller runs four safety checks every loop tick (10 ms), in priority order:

```
1. Disarmed?
   └─ YES → motors to 1000 µs, reset all PIDs, skip flight math

2. IMU healthy? (last update within 200 ms)
   └─ NO  → immediate DISARM, motors to 1000 µs

3. Setpoint fresh? (UDP packet within 500 ms)
   └─ NO for 600 ms → DISARM, motors to 1000 µs

4. Throttle below idle (< 1080 µs)?
   └─ YES → motors to 1050 µs, reset PID integrators
```

Additional safety guards:

- **MQTT disconnect while armed** → immediate disarm
- **Arm refused** if IMU is unhealthy or not yet calibrated
- **ESC calibration refused** if armed
- **Motor test refused** if armed

---

## Project Roadmap

### Immediate (Phase 4)

- [ ] Battery voltage monitoring via ADC1 (resistor divider → any GPIO1–GPIO10)
- [ ] Barometer integration (BMP388 or DPS310 via I2C) for altitude hold
- [ ] Persistent PID gain storage in NVS (survive power cycles)

### Medium Term (Phase 5)

- [ ] SIM7670G GNSS integration via UART (AT command mode)
- [ ] GPS position reporting to Traccar via OsmAnd HTTP GET protocol
- [ ] A1 Macedonia cellular: APN `internet`, MCC=294, MNC=03
- [ ] FreeRTOS task for GNSS polling (separate from IMU and flight control)

### Long Term

- [ ] DShot ESC protocol via RMT peripheral (eliminate ESC calibration, add telemetry)
- [ ] Magnetometer (HMC5883L) for yaw heading hold
- [ ] nRF24L01 or ELRS radio link for sub-5 ms control latency
- [ ] Optical flow sensor for indoor position hold without GPS

---

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

<div align="center">

Built with [ESP-IDF 5.5.1](https://github.com/espressif/esp-idf) · Targeting [ESP32-S3](https://www.espressif.com/en/products/socs/esp32-s3)

</div>
