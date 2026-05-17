# HAWK 🦅
### Hand Actuated Wireless Kinetics for UAV Navigation

> A wearable gesture-based control system for real-time, hands-free navigation of multiple UAVs using a smart glove.

## Overview

HAWK eliminates traditional RC controllers by translating natural hand gestures into precise drone flight commands. A smart glove equipped with an MPU6050 IMU captures hand orientation and transmits flight commands wirelessly via ESP-NOW to two independent drones. Each drone runs a dedicated ESP32 receiver that communicates with a Betaflight flight controller using the MSP protocol.

Tilt your hand forward — the drone pitches forward. Tilt left — it banks left. Touch a wire to ground — it arms and lifts off. Release — it lands softly.

---

## Demo

> Add your flight video here

---

## System Architecture

```
┌─────────────────────────────┐         ┌──────────────────────────────┐
│         GLOVE UNIT          │         │          DRONE UNIT          │
│                             │         │                              │
│  MPU6050 IMU                │         │  ESP32 Receiver              │
│  (pitch + roll angles)      │         │  (ESP-NOW → MSP decode)      │
│           ↓                 │  ESP-NOW│           ↓                  │
│  ESP32 Microcontroller ─────┼────────▶│  F722 Flight Controller      │
│  (filter + map + transmit)  │ 5-10ms  │  (Betaflight Angle Mode)     │
│           ↓                 │         │           ↓                  │
│  Conductive Probe Trigger   │         │  SimonK 30A ESCs             │
│  (drone selection + arm)    │         │           ↓                  │
│                             │         │  A2212 2200KV BLDC Motors    │
└─────────────────────────────┘         └──────────────────────────────┘
```

---

## Hardware

### Glove Unit
| Component       | Details                                    |
|-----------------|--------------------------------------------|
| Microcontroller | ESP32 (38-pin)                             |
| IMU             | MPU6050 (I²C)                              |
| Drone trigger   | Conductive probe wires (GPIO 13 + GPIO 14) |
| Power           | USB / LiPo                                |

### Drone Unit (x2)
| Component         | Details                    |
|-------------------|----------------------------|
| Flight Controller | Radiolink F722 (STM32F722) |
| Receiver          | ESP32 (UART2 → MSP)        |
| ESCs              | SimonK 30A (PWM)           |
| Motors            | A2212 2200KV BLDC          |
| Props             | 5152 tri-blade             |
| Battery           | 3S LiPo                    |
| Frame             | 450mm quad                 |

---

## Wiring

### Glove ESP32
```
MPU6050 SDA  →  GPIO 21
MPU6050 SCL  →  GPIO 22
MPU6050 VCC  →  3.3V
MPU6050 GND  →  GND

Drone 1 wire →  GPIO 13 (touch to GND = arm drone 1)
Drone 2 wire →  GPIO 14 (touch to GND = arm drone 2)
```

### Drone ESP32 → F722 FC
```
ESP32 TX (GPIO 17)  →  F722 UART2 RX
ESP32 RX (GPIO 16)  →  F722 UART2 TX
ESP32 GND           →  F722 GND
ESP32 5V            →  F722 5V
```

---

## Software Setup

### Prerequisites
- Arduino IDE 2.x
- ESP32 board package installed
- Libraries: `MPU6050`, `esp_now`, `WiFi`, `HardwareSerial`

### Glove (Transmitter)

1. Open `glove/transmitter_glove.ino`
2. Update MAC addresses to match your drone ESP32s:
```cpp
uint8_t drone1MAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // replace
uint8_t drone2MAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // replace
```
3. Flash to glove ESP32
4. Open Serial Monitor at 115200 baud
5. Note the transmitter MAC address printed on boot

### Drone (Receiver)

1. Open `drone/receiver_final.ino`
2. Update trusted sender MAC to match your glove ESP32:
```cpp
uint8_t trustedSender[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // replace
```
3. Tune hover throttle for your drone:
```cpp
#define THROTTLE_HOVER  1350  // increase until drone hovers
```
4. Flash to drone ESP32 (disconnect from FC first)

### Betaflight Configuration

| Setting | Value |
|---|---|
| Receiver mode | MSP (control via MSP port) |
| UART2 | Configuration/MSP enabled |
| ESC Protocol | PWM |
| Motor PWM rate | 400Hz |
| Flight mode | Angle |
| Angle limit | 45° |
| Angle strength | 80 |
| Motor output limit | 100% |
| ARM channel | AUX1 → 1600-2100 |
| Angle mode channel | AUX2 → 1600-2100 |

**In CLI:**
```
set motor_pwm_rate = 400
save
```

**PID values (tuned for calm flight):**
|       | P  | I  | D  |
|-------|----|----|----|
| Roll  | 32 | 30 | 18 |
| Pitch | 35 | 28 | 20 |
| Yaw   | 30 | 40 | 0  |

---

## How It Works

### Gesture to Flight Command Pipeline

**1. Sensing**
The MPU6050 IMU on the back of the glove continuously measures accelerometer data on 3 axes at 50Hz.

**2. Angle Calculation**
Pitch and roll angles are computed using arctangent approximations:
```
pitch = atan2(ay, az) × (180 / π)
roll  = atan2(ax, az) × (180 / π)
```

**3. Filtering**
An exponential smoothing filter (α = 0.15) removes sensor noise. A ±5° deadzone eliminates natural hand tremor.

**4. RC Mapping**
Filtered angles (-45° to +45°) are mapped linearly to RC channel values (1350 to 1650 µs), centered at 1500µs neutral.

**5. Transmission**
A `ControlPacket` struct is transmitted via ESP-NOW unicast to the selected drone at 50Hz. The inactive drone always receives a disarm packet.

**6. Flight Control**
The drone ESP32 decodes packets and sends MSP commands to the F722 FC at 50Hz. A 3-state firmware state machine manages:
- `ARMING` — throttle ramps from 1000 to hover over ~1.75 seconds
- `FLYING` — gesture commands active, self-leveling angle mode
- `LANDING` — throttle ramps down over ~3.5 seconds on wire release

---

## Control Mapping

| Gesture             | Drone Action      |
|---------------------|-------------------|
| Tilt hand forward   | Pitch forward     |
| Tilt hand backward  | Pitch backward    |
| Tilt hand left      | Roll left         |
| Tilt hand right     | Roll right        |
| Touch Wire 1 to GND | Arm + fly Drone 1 |
| Touch Wire 2 to GND | Arm + fly Drone 2 |
| Release wire        | Soft landing      |

---

## Results

| Metric                         | Value                 |
|--------------------------------|-----------------------|
| ESP-NOW transmission latency   | 5–10 ms               |
| Packet delivery rate           | 100%                  |
| Transmission frequency         | 50 Hz                 |
| Communication range (indoor)   | Up to 10 m            |
| Gesture deadzone               | ±5 degrees            |
| Throttle ramp-up duration      | ~1.75 seconds         |
| Throttle ramp-down duration    | ~3.5 seconds          |
| Flight mode                    | Angle (self-leveling) |
| FC protocol                    | MSP over UART         |

---

## Repository Structure

```
HAWK/
├── README.md
├── glove/
│   └── transmitter_glove.ino
├── drone/
│   └── receiver_final.ino
├── docs/
│   ├── architecture_diagram.png
│   ├── betaflight_config.md
│   └── wiring_diagram.png
└── paper/
    ├── HAWK_Paper.pdf
    └── HAWK_Presentation.pptx
```

---

## Paper

**Hand Actuated Wireless Kinetics for UAV Navigation**
Presented at the 14th International Conference on Current Engineering Trends (ICCET 2026)
Paper ID: ICCET267691

---

## Authors

- **Amith Mathew** — amith7mathew@gmail.com
- **Vaishnavi Nambiar** — vaishnavinambiar25@gmail.com
- **Alna George** — alnageorge11@gmail.com
- **Anjaleena Joseph** — anjaleenajoseph0323@gmail.com

**Guide:** Ms. Unnimaya M U, Assistant Professor
Department of Computer Science and Engineering
Vimal Jyothi Engineering College, Chemperi, Kannur, Kerala

---

## License

MIT License — free to use, modify and distribute with attribution.

gloves

<img width="864" height="1152" alt="gloves" src="https://github.com/user-attachments/assets/b7c3fcc4-f0c4-4669-bf65-e0eae4365051" />


drone

<img width="3119" height="4160" alt="drone" src="https://github.com/user-attachments/assets/2113ea5a-1e36-436b-b2dd-82c57ac7b121" />

equipment

<img width="3119" height="4160" alt="equpiment" src="https://github.com/user-attachments/assets/0acba652-85f1-4e42-bae0-fc77a22a8e5d" />


