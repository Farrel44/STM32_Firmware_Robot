#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Motor PWM output for BTS7960-style dual-PWM inputs (RPWM/LPWM).
 *
 * Why a dedicated module:
 * - Centralizes timer/channel mapping so control code stays readable.
 * - Ensures safe ordering (set duty=0 before enabling the driver).
 */

void MotorPwm_Init(void);

/* Enable/disable the motor driver (global enable pin). */
void MotorPwm_SetEnabled(bool enabled);

/*
 * Set motor command in ESP-compatible units (-255..255).
 * Positive => RPWM active, negative => LPWM active.
 */
void MotorPwm_Set(int motor_index_1based, int16_t pwm);

/* Convenience: force all channels to zero (does not change enable state). */
void MotorPwm_StopAll(void);

#endif /* MOTOR_PWM_H */
