#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// ===== User-tunable parameters =====
#define MIN_DUTY 0.00f
#define MAX_DUTY 1.00f

// Motor selection
enum MotorCh {
    MD1 = 1,
    MD2 = 2
};

// Stop behavior when duty = 0
enum StopMode {
    STOP_COAST = 0,  // L/L (Hi-Z, low power, coasting)
    STOP_BRAKE = 1   // H/H (active brake, quick stop)
};

// Init PWM on pins/channels
void MotorDriver_begin();

// Set motor speed in range -1.0 .. +1.0
//  >0: forward  (slow-decay PWM: drive <-> brake)
//  <0: reverse  (slow-decay PWM: drive <-> brake)
// ==0: stop     (obeys current StopMode)
void MotorDriver_setSpeed(MotorCh ch, float dutyRatio);

// Set/Get stop mode (takes effect whenever duty becomes 0)
void MotorDriver_setStopMode(StopMode mode);
StopMode MotorDriver_getStopMode();

#endif // MOTOR_DRIVER_H
