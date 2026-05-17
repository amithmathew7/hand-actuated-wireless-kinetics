/*
 * RECEIVER – ESP32 → BETAFLIGHT MSP
 * Fixed hover throttle. Pitch + Roll only.
 * Smooth ramp UP on arm, smooth ramp DOWN on disarm.
 */

#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

// ============= PINS =============
#define FC_RX_PIN 16
#define FC_TX_PIN 17

// ============= TUNE THESE =============

// Target hover throttle — increase by 25 each flash until drone hovers
#define THROTTLE_HOVER      1325

// Ramp UP speed — how fast throttle climbs on arm
// 2 = ~1.75 seconds from 1000 to 1350 (safe, recommended)
// 4 = ~0.9 seconds (faster but more lurch)
#define TAKEOFF_RAMP_RATE   2

// Ramp DOWN speed — how fast throttle drops on disarm
// 2 = ~3.5 seconds from 1350 to 1000 (gentle landing)
// 4 = ~1.75 seconds (faster landing)
#define LANDING_RAMP_RATE   5

// ======================================

#define THROTTLE_MIN        1000

#define ARM_VALUE           1800
#define DISARM_VALUE        1000
#define ANGLE_MODE_ON       1800
#define ANGLE_MODE_OFF      1000

#define MAX_ROLL_OFFSET     120
#define MAX_PITCH_OFFSET    120
#define DEADZONE            10

#define FAILSAFE_TIMEOUT    500
#define MSP_SET_RAW_RC      200

// ============= STATES =============
typedef enum {
  STATE_DISARMED,   // wire not touched, motors off
  STATE_ARMING,     // wire just touched, ramping throttle UP
  STATE_FLYING,     // hovering at THROTTLE_HOVER, gestures active
  STATE_LANDING     // wire released, ramping throttle DOWN
} DroneState;

// ============= TRUSTED TRANSMITTER MAC =============
uint8_t trustedSender[] = {0x6C, 0xC8, 0x40, 0x05, 0x98, 0x48};

// ============= PACKET =============
typedef struct {
  bool     fly_active;
  uint16_t throttle;
  uint16_t pitch;
  uint16_t roll;
  uint16_t yaw;
} ControlPacket;

ControlPacket incoming;

// ============= GLOBALS =============
HardwareSerial FC(2);
uint16_t rc[8];
unsigned long lastRecvTime = 0;
bool flyCommand     = false;
DroneState state    = STATE_DISARMED;
float rampThrottle  = THROTTLE_MIN;

// ============= ESP-NOW CALLBACK =============
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (memcmp(mac, trustedSender, 6) != 0) return;
  if (len != sizeof(ControlPacket)) return;

  memcpy(&incoming, data, sizeof(incoming));
  flyCommand   = incoming.fly_active;
  lastRecvTime = millis();
}

// ============= MSP =============
void sendMSP(uint8_t cmd, uint8_t *payload, uint8_t size) {
  uint8_t checksum = 0;
  FC.write('$'); FC.write('M'); FC.write('<');
  FC.write(size);  checksum ^= size;
  FC.write(cmd);   checksum ^= cmd;
  for (int i = 0; i < size; i++) {
    FC.write(payload[i]);
    checksum ^= payload[i];
  }
  FC.write(checksum);
}

void sendRC() {
  uint8_t payload[16];
  for (int i = 0; i < 8; i++) {
    payload[i * 2]     = rc[i] & 0xFF;
    payload[i * 2 + 1] = rc[i] >> 8;
  }
  sendMSP(MSP_SET_RAW_RC, payload, 16);
}

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  FC.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  lastRecvTime = millis();
  Serial.println("Receiver Ready");
}

// ============= LOOP =============
void loop() {
  unsigned long now = millis();

  // Failsafe
  if (now - lastRecvTime > FAILSAFE_TIMEOUT)
    flyCommand = false;

  // ---- State machine ----
  switch (state) {

    case STATE_DISARMED:
      rampThrottle = THROTTLE_MIN;
      if (flyCommand) {
        state = STATE_ARMING;
        Serial.println("→ ARMING (ramp up)");
      }
      break;

    case STATE_ARMING:
      if (!flyCommand) {
        // wire released before fully armed — go straight to landing
        state = STATE_LANDING;
        Serial.println("→ LANDING (released during arming)");
      } else {
        rampThrottle += TAKEOFF_RAMP_RATE;
        if (rampThrottle >= THROTTLE_HOVER) {
          rampThrottle = THROTTLE_HOVER;
          state        = STATE_FLYING;
          Serial.println("→ FLYING");
        }
      }
      break;

    case STATE_FLYING:
      rampThrottle = THROTTLE_HOVER;
      if (!flyCommand) {
        state = STATE_LANDING;
        Serial.println("→ LANDING (ramp down)");
      }
      break;

    case STATE_LANDING:
      if (flyCommand) {
        // wire touched again mid-landing — go back to arming
        state = STATE_ARMING;
        Serial.println("→ ARMING (re-triggered during landing)");
      } else {
        rampThrottle -= LANDING_RAMP_RATE;
        if (rampThrottle <= THROTTLE_MIN) {
          rampThrottle = THROTTLE_MIN;
          state        = STATE_DISARMED;
          Serial.println("→ DISARMED (landed)");
        }
      }
      break;
  }

  // ---- Safe defaults ----
  rc[0] = 1500;
  rc[1] = 1500;
  rc[2] = THROTTLE_MIN;
  rc[3] = 1500;
  rc[4] = DISARM_VALUE;
  rc[5] = ANGLE_MODE_OFF;
  rc[6] = 1000;
  rc[7] = 1000;

  // ---- Apply state ----
  if (state == STATE_ARMING) {
    // Keep armed, level, throttle ramping up
    rc[4] = ARM_VALUE;
    rc[5] = ANGLE_MODE_ON;
    rc[2] = (uint16_t)rampThrottle;
    rc[0] = 1500;
    rc[1] = 1500;
    Serial.printf("ARMING  | Thr:%.0f\n", rampThrottle);
  }

  else if (state == STATE_FLYING) {
    rc[4] = ARM_VALUE;
    rc[5] = ANGLE_MODE_ON;
    rc[2] = THROTTLE_HOVER;

    int rollOffset  = constrain(incoming.roll  - 1500, -MAX_ROLL_OFFSET,  MAX_ROLL_OFFSET);
    int pitchOffset = constrain(incoming.pitch - 1500, -MAX_PITCH_OFFSET, MAX_PITCH_OFFSET);

    if (abs(rollOffset)  < DEADZONE) rollOffset  = 0;
    if (abs(pitchOffset) < DEADZONE) pitchOffset = 0;

    rc[0] = 1500 + rollOffset;
    rc[1] = 1500 + pitchOffset;

    Serial.printf("FLYING  | Thr:%d  R:%d  P:%d\n", rc[2], rc[0], rc[1]);
  }

  else if (state == STATE_LANDING) {
    // Stay armed, level, throttle ramping down
    rc[4] = ARM_VALUE;
    rc[5] = ANGLE_MODE_ON;
    rc[2] = (uint16_t)rampThrottle;
    rc[0] = 1500;
    rc[1] = 1500;
    Serial.printf("LANDING | Thr:%.0f\n", rampThrottle);
  }

  sendRC();
  delay(20); // 50Hz
}
