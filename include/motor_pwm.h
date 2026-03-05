#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Motor PWM output for BTS7960-style dual-PWM inputs (RPWM/LPWM).
 *
 * Hardware architecture:
 * - All 3 BTS7960 modules share a single enable pin (PB0).
 *   Unlike the ESP32 design (3 separate EN pins), individual motor
 *   disable is only possible via PWM = 0, not EN toggle.
 * - Stop strategy: PWM = 0 with EN permanently HIGH (same as ESP32).
 *   This avoids BTS7960 EN rise-time glitches and simplifies safety logic.
 *
 * Timer mapping:
 *   Motor 1 → TIM1 CH1 (RPWM) / CH2 (LPWM)  [Advanced timer — requires MOE]
 *   Motor 2 → TIM1 CH3 (RPWM) / CH4 (LPWM)  [Advanced timer — requires MOE]
 *   Motor 3 → TIM2 CH1 (RPWM) / CH2 (LPWM)  [General-purpose timer]
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
