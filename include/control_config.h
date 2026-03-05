#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Control-loop configuration — single source of truth for PID, encoder,
 * feedforward, and motor output tuning.  All rate-dependent values are
 * scaled for the STM32's 200 Hz control tick (vs 100 Hz on ESP).
 */

/* Encoder: 7 PPR motor x 4 (quadrature) x 95/7 (gearbox) = 380 ticks/rev. */
#define CONTROL_TICKS_PER_REV            (380.0f)

/* Encoder direction inversion — compensate mounting/wiring polarity. */
#define CONTROL_INVERT_ENC1              (false)
#define CONTROL_INVERT_ENC2              (false)
#define CONTROL_INVERT_ENC3              (true)

/* PWM direction inversion — compensate RPWM/LPWM wiring. */
#define CONTROL_INVERT_PWM1              (false)
#define CONTROL_INVERT_PWM2              (false)
#define CONTROL_INVERT_PWM3              (false)

/* PID gains (process variable: RPM). */
#define CONTROL_PID_KP                   (0.40f)
#define CONTROL_PID_KI                   (3.60f)
#define CONTROL_PID_KD                   (0.04f)

/* Integral anti-windup limit. */
#define CONTROL_PID_INTEGRAL_LIMIT       (50.0f)

/* PWM output range (0..255), mapped to timer CCR by motor_pwm module. */
#define CONTROL_PWM_MAX                  (255)

/* Feedforward linearisation: PWM = OFFSET + SLOPE x |RPM|. */
#define CONTROL_FF_SLOPE                 (0.3f)
#define CONTROL_FF_OFFSET                (5.0f)
#define CONTROL_FF_DEADBAND_RPM          (3.0f)

/* Dead-zone — reset PID integral when |target| is below this. */
#define CONTROL_DEAD_ZONE_RPM            (2.0f)

/* EMA filter coefficients for measured RPM (reduces derivative noise). */
#define CONTROL_EMA_ALPHA                (0.30f)
#define CONTROL_EMA_BETA                 (0.70f)

/* PWM rate limit per tick — 2 counts/5 ms = ESP's 4 counts/10 ms. */
#define CONTROL_PWM_RATE_LIMIT_PER_TICK  (2)

/* Maximum target RPM accepted from serial commands. */
#define CONTROL_MAX_RPM                  (600)

/* RPM ramp — halved from ESP (200 Hz tick vs 100 Hz) to keep same dRPM/dt. */
#define CONTROL_RAMP_UP_RATE             (2.5f)
#define CONTROL_RAMP_DOWN_RATE           (4.0f)

/* Snap-to-zero threshold for float residue in ramped setpoint. */
#define CONTROL_ZERO_THRESHOLD           (0.3f)

/* Complementary filter alpha — 70 % gyro / 30 % accel.
 * tau = dt/(1-a) ~ 33 ms  →  f_c ≈ 4.8 Hz.
 * Faster convergence to true tilt while still rejecting vibration
 * above ~5 Hz.  Previous 0.95 gave f_c ≈ 0.8 Hz — too sluggish
 * for rapid tilt changes common in omni-drive manoeuvres.
 */
#define CONTROL_IMU_FILTER_ALPHA         (0.70f)

/* Auto-disable IMU after this many consecutive I2C failures. */
#define CONTROL_IMU_ERROR_THRESHOLD      (10)

#endif /* CONTROL_CONFIG_H */
