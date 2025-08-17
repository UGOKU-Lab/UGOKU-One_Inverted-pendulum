#include "MotorDriver.h"
#include <math.h>

// ==== Pin mapping (UGOKU One assumed) ====
// MD1: IN1 = GPIO32, IN2 = GPIO33
// MD2: IN1 = GPIO5,  IN2 = GPIO13
static const int MD1_IN1 = 32;
static const int MD1_IN2 = 33;
static const int MD2_IN1 = 5;
static const int MD2_IN2 = 13;

// ==== PWM settings ====
static const int PWM_FREQ = 20000; // Hz
static const int PWM_RES  = 8;     // bits (0..255)

// LEDC channel numbers (ensure no conflicts elsewhere)
static const int CH_MD1_IN1 = 4;
static const int CH_MD1_IN2 = 5;
static const int CH_MD2_IN1 = 6;
static const int CH_MD2_IN2 = 7;

// Current global stop mode (default: low power)
static StopMode g_stopMode = STOP_COAST;

// ----- helpers -----
static inline uint8_t clamp8(int v) {
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

static inline uint8_t toDuty255(float x01) {
    // x01 expected in [0..1]
    return clamp8((int)lroundf(x01 * 255.0f));
}

static inline void getChannels(MotorCh ch, int& chIN1, int& chIN2) {
    if (ch == MD1) { chIN1 = CH_MD1_IN1; chIN2 = CH_MD1_IN2; }
    else           { chIN1 = CH_MD2_IN1; chIN2 = CH_MD2_IN2; }
}

// Apply stop according to g_stopMode
static void applyStop(MotorCh ch) {
    int chIN1, chIN2; getChannels(ch, chIN1, chIN2);
    if (g_stopMode == STOP_BRAKE) {
        // H/H (active brake)
        ledcWriteChannel(chIN1, 255);
        ledcWriteChannel(chIN2, 255);
    } else {
        // L/L (Hi-Z, coast)
        ledcWriteChannel(chIN1, 0);
        ledcWriteChannel(chIN2, 0);
    }
}

// ----- public API -----
void MotorDriver_begin() {
    // NOTE:
    // If your environment doesn't provide ledcAttachChannel / ledcWriteChannel,
    // replace with Arduino-ESP32 standard APIs: ledcSetup/ledcAttachPin/ledcWrite.

    // Attach PWM to each pin
    ledcAttachChannel(MD1_IN1, PWM_FREQ, PWM_RES, CH_MD1_IN1);
    ledcAttachChannel(MD1_IN2, PWM_FREQ, PWM_RES, CH_MD1_IN2);
    ledcAttachChannel(MD2_IN1, PWM_FREQ, PWM_RES, CH_MD2_IN1);
    ledcAttachChannel(MD2_IN2, PWM_FREQ, PWM_RES, CH_MD2_IN2);

    // Start in stop state
    applyStop(MD1);
    applyStop(MD2);
}

void MotorDriver_setStopMode(StopMode mode) {
    // Change the policy; it will apply the next time duty becomes 0
    g_stopMode = mode;
}

StopMode MotorDriver_getStopMode() {
    return g_stopMode;
}

void MotorDriver_setSpeed(MotorCh ch, float dutyRatio) {
    // Clamp input to -1.0 .. +1.0
    if (dutyRatio > 1.0f)  dutyRatio = 1.0f;
    if (dutyRatio < -1.0f) dutyRatio = -1.0f;

    // Stop: obey StopMode
    if (dutyRatio == 0.0f) {
        applyStop(ch);
        return;
    }

    // Map |input| from [0..1] to [MIN_DUTY..MAX_DUTY]
    float a = fabsf(dutyRatio);
    float mapped = MIN_DUTY + (MAX_DUTY - MIN_DUTY) * a;
    if (mapped < 0.0f) mapped = 0.0f;
    if (mapped > 1.0f) mapped = 1.0f;
    uint8_t duty = toDuty255(mapped);

    // Slow-decay (drive <-> brake) via single-side PWM:
    //  Forward: IN1 = HIGH (constant), IN2 = PWM (LOW=drive, HIGH=brake)
    //  Reverse: IN2 = HIGH (constant), IN1 = PWM (LOW=drive, HIGH=brake)
    int chIN1, chIN2; getChannels(ch, chIN1, chIN2);

    if (dutyRatio > 0.0f) {
        // Forward
        ledcWriteChannel(chIN1, 255);        // IN1 = H (constant)
        ledcWriteChannel(chIN2, 255 - duty); // IN2 PWM: LOW=drive, HIGH=brake
    } else {
        // Reverse
        ledcWriteChannel(chIN2, 255);        // IN2 = H (constant)
        ledcWriteChannel(chIN1, 255 - duty); // IN1 PWM: LOW=drive, HIGH=brake
    }
}
