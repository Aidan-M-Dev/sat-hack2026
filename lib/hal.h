#ifndef HAL_H
#define HAL_H

/*
 * lib/hal.h
 *
 * Hardware Abstraction Layer.  Provides basic wrappers
 * over Arduino pins so the rest of the codebase does not touch pins
 * directly.
 *
 * Functions are `inline` so the resulting code is header-only and
 * compatible with the Arduino IDE's single-file compilation model.
 */

#include <Arduino.h>

// pins
static const int PIN_MOTOR_L_FWD = 5;
static const int PIN_MOTOR_L_REV = 6;
static const int PIN_MOTOR_R_FWD = 9;
static const int PIN_MOTOR_R_REV = 10;
static const int PIN_TRIG        = A4;
static const int PIN_ECHO        = A5;

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------
inline void hal_init() {
    // motor pins
    pinMode(PIN_MOTOR_L_FWD, OUTPUT);
    pinMode(PIN_MOTOR_L_REV, OUTPUT);
    pinMode(PIN_MOTOR_R_FWD, OUTPUT);
    pinMode(PIN_MOTOR_R_REV, OUTPUT);

    // ultrasonic sensor pins
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);

    // ensure motors stopped
    analogWrite(PIN_MOTOR_L_FWD, 0);
    analogWrite(PIN_MOTOR_L_REV, 0);
    analogWrite(PIN_MOTOR_R_FWD, 0);
    analogWrite(PIN_MOTOR_R_REV, 0);
}

// ---------------------------------------------------------------------------
// Motor control
// ---------------------------------------------------------------------------
/**
 * hal_motor_left(fwd, rev)
 * Write PWM values to the left motor channels.  Each parameter should be
 * between 0 and 255; both should not be non-zero simultaneously
 */
inline void hal_motor_left(int fwd, int rev) {
    analogWrite(PIN_MOTOR_L_FWD, constrain(fwd, 0, 255));
    analogWrite(PIN_MOTOR_L_REV, constrain(rev, 0, 255));
}

/**
 * hal_motor_right(fwd, rev)
 */
inline void hal_motor_right(int fwd, int rev) {
    analogWrite(PIN_MOTOR_R_FWD, constrain(fwd, 0, 255));
    analogWrite(PIN_MOTOR_R_REV, constrain(rev, 0, 255));
}

// ---------------------------------------------------------------------------
// Sensor reading
// ---------------------------------------------------------------------------
/**
 * hal_ultrasonic_cm()
 * Perform a single ranging measurement with the HC-SR04 and return the
 * distance in centimetres.  If no echo is received within a reasonable
 * timeout the returned value will be large (~400).
 */
inline float hal_ultrasonic_cm() {
    long duration;

    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    duration = pulseIn(PIN_ECHO, HIGH, 30000); // 30ms timeout (~5m)
    if (duration == 0) {
        return 400.0f;
    }
    // speed of sound ~343 m/s = 0.0343 cm/us, divide by 2 for round-trip
    float cm = (duration * 0.0343f) / 2.0f;
    return cm;
}

inline unsigned long hal_millis() {
    return millis();
}

#endif
