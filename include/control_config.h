#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Control-loop configuration (ported from ESP_Robot_2026).
 *
 * Why keep these in one header:
 * - Single source of truth for calibration constants used by PID, RPM conversion, and motor output.
 * - Makes it obvious what must change when mechanical/hardware changes.
 */

/* Encoder calibration.
 *
 * ESP implementation uses full quadrature counting and derives RPM using:
 *   rpm = deltaTicks * (60 / (TICKS_PER_REV * dt))
 */
#define CONTROL_TICKS_PER_REV            (380.0f)

/* Direction calibration.
 * Why: mechanical mounting / wiring can flip the sign of encoder counts.
 */
#define CONTROL_INVERT_ENC1              (false)
#define CONTROL_INVERT_ENC2              (false)
#define CONTROL_INVERT_ENC3              (true)

/* PWM direction calibration.
 * Why: some drivers swap RPWM/LPWM semantics depending on wiring.
 */
#define CONTROL_INVERT_PWM1              (false)
#define CONTROL_INVERT_PWM2              (false)
#define CONTROL_INVERT_PWM3              (false)

/* PID tuning (same as ESP). Units assume RPM as process variable. */
#define CONTROL_PID_KP                   (0.40f)
#define CONTROL_PID_KI                   (3.60f)
#define CONTROL_PID_KD                   (0.04f)

/* PID anti-windup (same as ESP). */
#define CONTROL_PID_INTEGRAL_LIMIT       (50.0f)

/* Output range matches ESP (0..255). Motor driver maps it to timer duty cycle. */
#define CONTROL_PWM_MAX                  (255)

/* Feedforward: PWM = OFFSET + SLOPE * |RPM| (same as ESP). */
#define CONTROL_FF_SLOPE                 (0.3f)
#define CONTROL_FF_OFFSET                (5.0f)
#define CONTROL_FF_DEADBAND_RPM          (3.0f)

/* Dead-zone reset (same as ESP). */
#define CONTROL_DEAD_ZONE_RPM            (2.0f)

/* EMA filter constants (same as ESP). */
#define CONTROL_EMA_ALPHA                (0.30f)
#define CONTROL_EMA_BETA                 (0.70f)

/* PWM rate limit.
 *
 * ESP: 4 counts per 10ms (100Hz). Here control tick is 5ms (200Hz),
 * so we use 2 counts per tick to keep the same slope (counts/second).
 */
#define CONTROL_PWM_RATE_LIMIT_PER_TICK  (2)

/* RPM clamp: absolute maximum target RPM accepted from serial commands. */
#define CONTROL_MAX_RPM                  (600)

/* RPM ramp: smooth acceleration/deceleration to prevent wheel slip.
 *
 * ESP: 5.0 RPM/10ms (up), 8.0 RPM/10ms (down) at 100Hz.
 * STM32 runs at 200Hz (5ms tick), so rates are halved to match slope.
 */
#define CONTROL_RAMP_UP_RATE             (2.5f)
#define CONTROL_RAMP_DOWN_RATE           (4.0f)

/* Snap-to-zero threshold: avoid float residue keeping motors active. */
#define CONTROL_ZERO_THRESHOLD           (0.3f)

/* IMU complementary filter.
 *
 * Alpha = 0.95 means 95% gyro trust, 5% accelerometer trust.
 * Time constant tau = dt / (1 - alpha) = 0.01 / 0.05 = 0.2 seconds.
 *
 * Rationale:
 * - Gyro is accurate short-term but drifts over time.
 * - Accelerometer gives absolute reference but is noisy (motor vibration).
 * - 0.2s time constant filters vibration while staying responsive to real tilt.
 */
#define CONTROL_IMU_FILTER_ALPHA         (0.95f)

/* IMU error threshold: auto-disable after N consecutive I2C failures. */
#define CONTROL_IMU_ERROR_THRESHOLD      (10)

#endif /* CONTROL_CONFIG_H */
