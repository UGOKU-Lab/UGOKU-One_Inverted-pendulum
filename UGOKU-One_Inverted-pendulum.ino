// UGOKU Pad BLE controller
#include "UGOKU_Pad_Controller.hpp"

// Official Arduino IMU library for BMI270/BMM150
#include <Arduino_BMI270_BMM150.h>

// Motor driver
#include "MotorDriver.h"

// I2C
#include <Wire.h>
#include <math.h>
// ========= Configuration =========
// On-board LED debug (LOW=ON). IO2 for forward, IO4 for backward.
#define PIN_LED_FWD  2
#define PIN_LED_BWD  4

// Per-motor direction switches (ESP32 GPIO34/35 are input-only; no internal pullups)
// Wire with external pull-up/down so the pin is not floating.
#define PIN_MD1_DIR_SWITCH 34
#define PIN_MD2_DIR_SWITCH 35

// Complementary filter constant (0..1). Higher trusts gyro more.
static const float COMP_ALPHA = 0.985f;
// Fixed control loop period (seconds/us)
static const uint32_t CONTROL_PERIOD_US = 5000; // 200 Hz
static const float    CONTROL_PERIOD_S  = 0.005f;

// PID output limit -> maps to MotorDriver_setSpeed [-1..1]
static const float PID_OUT_LIMIT = 1.0f;

// Integral windup guard
static const float I_LIMIT = 2.0f;

// Minimum motor command to overcome deadzone (after scaling), 0..1
static const float MIN_DRIVE = 0.10f;
// Small error band (deg) where we cut output to reduce dithering
static const float ERROR_DEADBAND = 0.5f;

// ===== IMU axis/sign quick toggles =====
// If pitch方向や角速度の符号が合わない場合はここを変更
// Accel -> pitch tilt angle (deg)
#define PITCH_FROM_ACCEL(ax, ay, az) (atan2f((ax), (az)) * 180.0f / PI)
// Gyro -> pitch rate (deg/s)
#define PITCH_RATE_FROM_GYRO(gx, gy, gz)  ( -(gy) )

// UGOKU Pad channels for tuning
// Use the default sample’s sticks/sliders to adjust gains:
//  ch2 -> Kp, ch3 -> Ki, ch4 -> Kd, ch5 -> angle offset trim
static const uint8_t CH_KP     = 2;
static const uint8_t CH_KI     = 3;
static const uint8_t CH_KD     = 4;
static const uint8_t CH_ATRIM  = 5; // angle trim around 0 deg

// Map Pad value (fixed 0..180) to gain ranges
static inline float mapGain(uint8_t v, float maxGain) {
  return ((float)v) * (maxGain / 180.0f);
}

// Angle encode to 0..180 with 90 = 0deg
static inline uint8_t angleToByte180(float deg) {
  float v = 90.0f + deg; // -90..+90 -> 0..180
  if (v < 0) v = 0; if (v > 180) v = 180;
  return (uint8_t)(v + 0.5f);
}

UGOKU_Pad_Controller controller;

// IMU object from Arduino_BMI270_BMM150
// The library exposes a global-like instance name pattern; we use IMU directly.

// State
static bool isConnected = false;
static bool isArmed = false; // motors allowed to move only after Pad connects
static bool padReady = false; // first valid packet received

// PID gains (defaults)
static float Kp = 18.0f;
static float Ki = 0.0f;
static float Kd = 0.6f;

// Angle trim (deg)
static float angleTrim = 0.0f;

// Complementary filter state
static float pitchDeg = 0.0f;   // estimated
static uint32_t lastTickUs = 0; // control loop scheduler
static uint32_t lastTxMs  = 0;  // telemetry pacing (~50Hz)

// PID state
static float iTerm = 0.0f;
static float lastErr = 0.0f;

// Telemetry baseline (deg): measured at startup so upright displays 90 on ch20
static float telemetryZeroDeg = 0.0f;
// Gyro bias (deg/s) calibrated at startup
static float gyroBiasPitch = 0.0f;

// ch5 absolute trim: 90=>0° in 0..180 profile

// Forward declarations
void onDeviceConnect();
void onDeviceDisconnect();

void setup() {
  Serial.begin(115200);

  // I2C (ESP32 default: SDA=21, SCL=22)
  Wire.begin();

  // BLE
  controller.setup((char*)"UGOKU One");
  controller.setOnConnectCallback(onDeviceConnect);
  controller.setOnDisconnectCallback(onDeviceDisconnect);

  // Motors
  MotorDriver_begin();

  // LEDs
  pinMode(PIN_LED_FWD, OUTPUT);
  pinMode(PIN_LED_BWD, OUTPUT);
  digitalWrite(PIN_LED_FWD, HIGH); // OFF (LOW=ON)
  digitalWrite(PIN_LED_BWD, HIGH); // OFF

  // Per-motor direction switches (external pull-up/down required)
  pinMode(PIN_MD1_DIR_SWITCH, INPUT);
  pinMode(PIN_MD2_DIR_SWITCH, INPUT);

  // IMU init
  if (!IMU.begin()) {
    Serial.println("IMU begin failed (BMI270)");
  } else {
    Serial.println("IMU ready");
  }

  // Quick settle
  delay(200);

  // Rough initial pitch estimation from accelerometer (robot upright & still recommended)
  float ax, ay, az; float sum = 0; int n = 60;
  float gx, gy, gz; float gPitchSum = 0; int ng = 0;
  for (int i = 0; i < n; ++i) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      float p = PITCH_FROM_ACCEL(ax, ay, az);
      sum += p;
    }
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      gPitchSum += PITCH_RATE_FROM_GYRO(gx, gy, gz);
      ng++;
    }
    delay(5);
  }
  if (n > 0) pitchDeg = sum / (float)n;
  if (ng > 0) gyroBiasPitch = gPitchSum / (float)ng; else gyroBiasPitch = 0.0f;
  // Use the initial estimate as telemetry zero so upright shows 90 on the Pad
  telemetryZeroDeg = pitchDeg;
  lastTickUs = micros();
  lastTxMs  = millis();

  Serial.println("Waiting for UGOKU Pad connection...");
}

void onDeviceConnect() {
  Serial.println("Pad connected");
  isConnected = true;
  // Re-zero telemetry at connection so upright shows 90 on ch20
  telemetryZeroDeg = pitchDeg;
  // Wait until first valid Pad packet before arming
  padReady = false;
  isArmed = false;
}

void onDeviceDisconnect() {
  Serial.println("Pad disconnected");
  isConnected = false;
  isArmed = false; // disarm on disconnect
  // stop motors
  MotorDriver_setSpeed(MD1, 0);
  MotorDriver_setSpeed(MD2, 0);
}

static inline void updateFromPad() {
  // Read incoming 19B packet, ignore errors silently except for debug
  uint8_t err = controller.read_data();
  if (err != no_err) return;
  if (controller.getLastPairsCount() == 0) return;

  // Gains
  uint8_t vkp = controller.getDataByChannel(CH_KP);
  uint8_t vki = controller.getDataByChannel(CH_KI);
  uint8_t vkd = controller.getDataByChannel(CH_KD);
  uint8_t vat = controller.getDataByChannel(CH_ATRIM);

  // Treat 0xFF as no-data; clamp to 0..180; update only when valid
  bool have_kp = (vkp != 0xFF);
  bool have_ki = (vki != 0xFF);
  bool have_kd = (vkd != 0xFF);
  bool have_at = (vat != 0xFF);

  if (have_kp) { if (vkp > 180) vkp = 180; Kp = mapGain(vkp, 40.0f); }
  if (have_ki) { if (vki > 180) vki = 180; Ki = mapGain(vki, 2.0f); }
  if (have_kd) { if (vkd > 180) vkd = 180; Kd = mapGain(vkd, 2.0f); }

  // Angle trim: absolute trim (0..180 -> -90..+90)
  if (have_at) {
    if (vat > 180) vat = 180;
    angleTrim = (float)vat - 90.0f;
  }
  // Gentle clamp to avoid extreme targets causing safety cutoff
  if (angleTrim > 45.0f) angleTrim = 45.0f;
  if (angleTrim < -45.0f) angleTrim = -45.0f;

  // Arm once we have a first valid packet (at least trim and one gain)
  if (!padReady && (have_at || have_kp || have_kd || have_ki)) {
    padReady = true;
    isArmed = true;
  }
}

static inline void sendTelemetry() {
  // Send angles with 90 = baseline upright (connection baseline)
  uint8_t ch[9]; uint8_t val[9];
  for (int i = 0; i < 9; ++i) { ch[i] = 0xFF; val[i] = 0; }
  // Absolute display: do not subtract trim; upright => 90
  float dispDeg = pitchDeg - telemetryZeroDeg;
  ch[0] = 20; val[0] = angleToByte180(dispDeg);           // current angle
  ch[1] = 21; val[1] = angleToByte180(angleTrim);         // target angle (ch5), 90 when ch5=90
  // Kp telemetry on ch22 (scaled 0..40 -> 0..180 for visibility)
  ch[2] = 22; val[2] = (uint8_t)min(180.0f, max(0.0f, roundf(Kp * (180.0f / 40.0f))));
  // Ki and Kd telemetry (0..2 -> 0..180)
  ch[3] = 23; val[3] = (uint8_t)min(180.0f, max(0.0f, roundf(Ki * (180.0f / 2.0f))));
  ch[4] = 24; val[4] = (uint8_t)min(180.0f, max(0.0f, roundf(Kd * (180.0f / 2.0f))));
  controller.write_data(ch, val);
}

void loop() {
  // Update gains from Pad if connected
  if (isConnected) {
    updateFromPad();
  }

  // Fixed-period scheduler (200 Hz)
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastTickUs) < CONTROL_PERIOD_US) {
    // Pace telemetry even when not running control step
    uint32_t nowMsIdle = millis();
    if (nowMsIdle - lastTxMs > 20) { lastTxMs = nowMsIdle; if (isConnected) sendTelemetry(); }
    return;
  }
  // Catch up exactly one period (avoid drift); if lagged, step by multiples
  uint32_t elapsedUs = nowUs - lastTickUs;
  uint32_t steps = elapsedUs / CONTROL_PERIOD_US;
  lastTickUs += steps * CONTROL_PERIOD_US;
  const float dt = CONTROL_PERIOD_S;

  // Read sensors
  float ax, ay, az; float gx, gy, gz;
  bool haveAcc = IMU.accelerationAvailable();
  bool haveGyr = IMU.gyroscopeAvailable();
  if (haveAcc) IMU.readAcceleration(ax, ay, az);
  if (haveGyr) IMU.readGyroscope(gx, gy, gz);

  // Estimate pitch via complementary filter
  float pitchAcc = pitchDeg; // fallback
  if (haveAcc) {
    pitchAcc = PITCH_FROM_ACCEL(ax, ay, az);
  }
  // Bias-corrected gyro pitch rate (deg/s)
  float pitchRate = haveGyr ? (PITCH_RATE_FROM_GYRO(gx, gy, gz) - gyroBiasPitch) : 0.0f;
  float pitchGy = pitchDeg + (pitchRate * dt);
  pitchDeg = COMP_ALPHA * pitchGy + (1.0f - COMP_ALPHA) * pitchAcc;

  // Motor arming gate: do not drive before Pad connects
  if (!isArmed) {
    MotorDriver_setSpeed(MD1, 0);
    MotorDriver_setSpeed(MD2, 0);
    if (isConnected) sendTelemetry();
    delay(10);
    return;
  }

  // PID control using same baseline as telemetry
  // currentDeg: angle relative to connection baseline; targetDeg: ch5 absolute trim (90=>0°)
  float currentDeg = pitchDeg - telemetryZeroDeg;
  float targetDeg = angleTrim;
  float err = targetDeg - currentDeg;

  // Safety: if too far from upright, stop motors and reset integrator
  if (fabsf(err) > 60.0f) {
    MotorDriver_setSpeed(MD1, 0);
    MotorDriver_setSpeed(MD2, 0);
    iTerm = 0.0f;
  // LEDs off while out-of-range
  digitalWrite(PIN_LED_FWD, HIGH);
  digitalWrite(PIN_LED_BWD, HIGH);
    // don't drive until it is within range again
    if (isConnected) sendTelemetry();
    delay(10);
    return;
  }

  // Derivative from gyro rate improves noise immunity
  float dTerm = 0.0f;
  if (haveGyr) {
    dTerm = -Kd * pitchRate; // negative sign: if falling forward (rate>0), apply braking
  }
  lastErr = err;

  // Proportional + Integral + Derivative
  float uLinear = Kp * err + iTerm + dTerm;
  // Normalize to [-1..1] by a rough scale if needed
  // Here we assume Kp/Kd ranges chosen to keep |u| within a few tens
  // Scale down smartly:
  const float SCALE = 1.0f / 30.0f; // a bit stronger to catch falls faster
  float u = uLinear * SCALE;

  // Error deadband near target to prevent dithering
  if (fabsf(err) < ERROR_DEADBAND) {
    u = 0.0f;
    // bleed off integral slowly when close to target
    iTerm *= 0.98f;
  }

  // Deadzone compensation
  if (fabsf(u) > 0.0f && fabsf(u) < MIN_DRIVE) {
    u = (u > 0 ? MIN_DRIVE : -MIN_DRIVE);
  }

  // Saturate
  if (u > PID_OUT_LIMIT)  u = PID_OUT_LIMIT;
  if (u < -PID_OUT_LIMIT) u = -PID_OUT_LIMIT;

  // Conditional integrator (anti-windup): integrate only when not saturated
  if (fabsf(u) < (PID_OUT_LIMIT * 0.98f)) {
    iTerm += err * dt * Ki;
  }
  // Clamp integral
  if (iTerm > I_LIMIT) iTerm = I_LIMIT;
  if (iTerm < -I_LIMIT) iTerm = -I_LIMIT;

  // LED debug: turn on IO2 when leaning forward and commanding forward,
  // IO4 when leaning backward and commanding backward (LOW=ON).
  bool forwardTilt  = (currentDeg < (targetDeg - 0.5f));   // small deadband
  bool backwardTilt = (currentDeg > (targetDeg + 0.5f));
  if (forwardTilt && u > 0.0f) {
    digitalWrite(PIN_LED_FWD, LOW);
  } else {
    digitalWrite(PIN_LED_FWD, HIGH);
  }
  if (backwardTilt && u < 0.0f) {
    digitalWrite(PIN_LED_BWD, LOW);
  } else {
    digitalWrite(PIN_LED_BWD, HIGH);
  }

  // Per-motor direction factors (HIGH=normal, LOW=invert)
  int md1SwitchFactor = (digitalRead(PIN_MD1_DIR_SWITCH) == HIGH) ? 1 : -1;
  int md2SwitchFactor = (digitalRead(PIN_MD2_DIR_SWITCH) == HIGH) ? 1 : -1;

  // Drive both wheels same direction to balance
  MotorDriver_setSpeed(MD1, (float)(md1SwitchFactor) * u);
  MotorDriver_setSpeed(MD2, (float)(md2SwitchFactor) * u);

  // Send telemetry at ~50Hz
  uint32_t nowMs = millis();
  if (nowMs - lastTxMs > 20) { lastTxMs = nowMs; if (isConnected) sendTelemetry(); }
}
