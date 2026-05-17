# hand-actuated-wireless-kinetics

HAWK — Hand Actuated Wireless Kinetics for UAV Navigation
HAWK is a wearable gesture-based UAV control system that replaces traditional RC controllers with natural hand movements. A smart glove equipped with an ESP32 microcontroller and MPU6050 IMU captures hand orientation in real time. Pitch and roll angles are extracted from accelerometer data, filtered using exponential smoothing, and mapped to RC channel values. A conductive probe trigger on the glove selects and arms individual drones.
Commands are transmitted wirelessly via ESP-NOW at 50Hz, achieving 5–10ms end-to-end latency with 100% packet delivery in indoor testing up to 10 meters. Each drone runs a dedicated ESP32 receiver that decodes incoming packets and communicates with a Radiolink F722 flight controller using the MultiWii Serial Protocol (MSP) over UART at 115200 baud.
The flight controller runs Betaflight in Angle mode, providing self-leveling behavior that complements gesture-based control. A 3-state firmware state machine manages smooth throttle ramp-up on arming (~1.75 seconds), stable gesture-controlled flight, and gradual throttle ramp-down on disarm (~3.5 seconds), eliminating dangerous motor surges. A 500ms failsafe timeout automatically disarms the drone on signal loss.
Hardware Stack:

Glove: ESP32 · MPU6050 IMU · Conductive probe trigger
Drones: Radiolink F722 FC · SimonK 30A ESCs · A2212 2200KV BLDC motors · 5152 tri-blade props
Communication: ESP-NOW (2.4GHz integrated RF) → MSP over UART

Software Stack:

Embedded C++ on Arduino framework
Betaflight firmware with custom MSP integration
ESP-NOW peer-to-peer protocol
Exponential smoothing filter with deadzone compensation

Key Results:

ESP-NOW latency: 5–10ms
Packet delivery rate: 100%
Transmission frequency: 50Hz
Dual drone selection and independent control
Self-leveling stable hover achieved

Presented at the 14th International Conference on Current Engineering Trends (ICCET 2026), Paper ID: ICCET267691.
