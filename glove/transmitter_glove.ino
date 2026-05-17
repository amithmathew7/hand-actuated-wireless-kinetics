/*
 * TRANSMITTER – GESTURE DRONE GLOVE
 * Pitch + Roll only. Fixed hover throttle.
 *
 * Pin 13 → touch to GND = control Drone 1
 * Pin 14 → touch to GND = control Drone 2
 *
 * Tilt hand forward/back  → Pitch
 * Tilt hand left/right    → Roll
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>

// ============= MAC ADDRESSES =============
uint8_t drone1MAC[] = {0x00, 0x4B, 0x12, 0x34, 0xA4, 0x1C};
uint8_t drone2MAC[] = {0x20, 0xE7, 0xC8, 0xAD, 0xAC, 0xF4};

// ============= PINS =============
#define TRIGGER_PIN_1 13
#define TRIGGER_PIN_2 14

// ============= TUNING =============
// How far you tilt = how much drone responds
// Higher = more sensitive
#define TILT_SENSITIVITY  150

// Smoothing — higher = smoother but more lag
// Range: 0.1 (very smooth) to 0.5 (more responsive)
#define SMOOTHING         0.15

// Tilt deadzone in degrees — filters hand tremor
#define TILT_DEADZONE     5

// ============= PACKET =============
typedef struct {
  bool     fly_active;
  uint16_t throttle;  // not used by receiver, kept for struct match
  uint16_t pitch;
  uint16_t roll;
  uint16_t yaw;       // not used by receiver, kept for struct match
} ControlPacket;

// ============= GLOBALS =============
MPU6050 mpu;
esp_now_peer_info_t peer;
ControlPacket pkt;

float filteredPitch = 0;
float filteredRoll  = 0;

// ============= SETUP =============
void setup() {
  Serial.begin(115200);

  pinMode(TRIGGER_PIN_1, INPUT_PULLUP);
  pinMode(TRIGGER_PIN_2, INPUT_PULLUP);

  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection())
    Serial.println("MPU OK");
  else
    Serial.println("MPU FAIL — check wiring");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Transmitter MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  peer.channel = 0;
  peer.encrypt = false;

  memcpy(peer.peer_addr, drone1MAC, 6);
  esp_now_add_peer(&peer);

  memcpy(peer.peer_addr, drone2MAC, 6);
  esp_now_add_peer(&peer);

  Serial.println("Glove Ready");
}

// ============= LOOP =============
void loop() {
  bool activeD1 = (digitalRead(TRIGGER_PIN_1) == LOW);
  bool activeD2 = (digitalRead(TRIGGER_PIN_2) == LOW);
  bool anyActive = activeD1 || activeD2;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  if (anyActive) {
    // ---- Pitch & Roll from accelerometer ----
    float rawPitch = atan2(ay, az) * 180.0 / PI;
    float rawRoll  = atan2(ax, az) * 180.0 / PI;

    // Smooth the values
    filteredPitch = filteredPitch * (1.0 - SMOOTHING) + rawPitch * SMOOTHING;
    filteredRoll  = filteredRoll  * (1.0 - SMOOTHING) + rawRoll  * SMOOTHING;

    // Deadzone — ignore small tremors
    if (abs(filteredPitch) < TILT_DEADZONE) filteredPitch = 0;
    if (abs(filteredRoll)  < TILT_DEADZONE) filteredRoll  = 0;

    // Map tilt angle (-45 to +45 deg) to RC value (1350 to 1650)
    int pitchRC = map(
      constrain((int)filteredPitch, -45, 45),
      -45, 45,
      1500 - TILT_SENSITIVITY,
      1500 + TILT_SENSITIVITY
    );

    int rollRC = map(
      constrain((int)filteredRoll, -45, 45),
      -45, 45,
      1500 - TILT_SENSITIVITY,
      1500 + TILT_SENSITIVITY
    );

    pkt.fly_active = true;
    pkt.throttle   = 1500;  // ignored by receiver, FC uses fixed hover
    pkt.pitch      = pitchRC;
    pkt.roll       = rollRC;
    pkt.yaw        = 1500;  // centered, no yaw control

    Serial.printf("P:%d  R:%d  (raw pitch:%.1f  roll:%.1f)\n",
                  pitchRC, rollRC, filteredPitch, filteredRoll);

  } else {
    // No trigger — send stop packet
    filteredPitch = 0;
    filteredRoll  = 0;

    pkt.fly_active = false;
    pkt.throttle   = 1000;
    pkt.pitch      = 1500;
    pkt.roll       = 1500;
    pkt.yaw        = 1500;
  }

  // ---- Send to active drone ----
  if (activeD1) {
    esp_now_send(drone1MAC, (uint8_t*)&pkt, sizeof(pkt));
    Serial.println("→ Drone 1");
  } else {
    ControlPacket stop = {false, 1000, 1500, 1500, 1500};
    esp_now_send(drone1MAC, (uint8_t*)&stop, sizeof(stop));
  }

  if (activeD2) {
    esp_now_send(drone2MAC, (uint8_t*)&pkt, sizeof(pkt));
    Serial.println("→ Drone 2");
  } else {
    ControlPacket stop = {false, 1000, 1500, 1500, 1500};
    esp_now_send(drone2MAC, (uint8_t*)&stop, sizeof(stop));
  }

  delay(20); // 50Hz
}
 
