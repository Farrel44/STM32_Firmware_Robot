#include "motor_pwm.h"

#include "control_config.h"

#include "gpio.h"
#include "tim.h"

#include <stdlib.h>

static uint32_t clamp_u32(uint32_t x, uint32_t max)
{
  return (x > max) ? max : x;
}

static uint32_t pwm255_to_ccr(const TIM_HandleTypeDef *htim, uint16_t pwm_abs)
{
  const uint32_t arr = __HAL_TIM_GET_AUTORELOAD((TIM_HandleTypeDef *)htim);
  const uint32_t scaled = ((uint32_t)pwm_abs * (arr + 1U) + (CONTROL_PWM_MAX - 1U)) / CONTROL_PWM_MAX;
  if (scaled == 0U)
  {
    return 0U;
  }
  return clamp_u32(scaled - 1U, arr);
}

static void set_dual_pwm(TIM_HandleTypeDef *htim, uint32_t ch_rpwm, uint32_t ch_lpwm, int16_t pwm)
{
  int16_t cmd = pwm;
  if (cmd > CONTROL_PWM_MAX)
  {
    cmd = CONTROL_PWM_MAX;
  }
  else if (cmd < -CONTROL_PWM_MAX)
  {
    cmd = -CONTROL_PWM_MAX;
  }

  uint16_t mag = (uint16_t)abs((int)cmd);
  const uint32_t ccr = pwm255_to_ccr(htim, mag);

  /*
   * SAFETY: always write the INACTIVE channel to 0 BEFORE activating the
   * other channel.  Without CCR preload, register writes take effect
   * immediately on the running counter comparator.  If we wrote the active
   * channel first, there would be a brief window where BOTH RPWM and LPWM
   * have non-zero CCR — which causes cross-conduction in the BTS7960
   * dual half-bridge.
   */
  if (cmd >= 0)
  {
    __HAL_TIM_SET_COMPARE(htim, ch_lpwm, 0);
    __HAL_TIM_SET_COMPARE(htim, ch_rpwm, ccr);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(htim, ch_rpwm, 0);
    __HAL_TIM_SET_COMPARE(htim, ch_lpwm, ccr);
  }
}

/*
 * Start all PWM channels at 0 % duty, enable TIM1 main output, then
 * assert the driver enable pin.
 *
 * Sequence:
 *   1. HAL_TIM_PWM_Start all 6 channels (CCR pre-loaded to 0 by CubeMX).
 *   2. Explicitly set MOE in TIM1->BDTR — required for advanced timers to
 *      propagate OCx compare matches to the physical pins.
 *      HAL_TIM_PWM_Start does this internally, but an explicit call makes
 *      the dependency self-documenting and guards against HAL quirks.
 *   3. Zero all CCR as a belt-and-suspenders measure.
 *   4. Assert EN_MOTOR HIGH — same strategy as ESP32 (stop = PWM 0, not
 *      EN toggle) to avoid BTS7960 EN rise-time glitches.
 */
void MotorPwm_Init(void)
{
  /* --- Start PWM channels (CCR = 0, no output yet) --- */
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /*
   * CRITICAL: TIM1 is an advanced timer — its outputs stay tri-stated until
   * the Main Output Enable (MOE) bit in BDTR is set.  Without this, motors
   * 1 & 2 (TIM1 CH1-CH4) will never see PWM regardless of CCR values.
   * TIM2 is a general-purpose timer and has no MOE requirement.
   */
  __HAL_TIM_MOE_ENABLE(&htim1);

  MotorPwm_StopAll();
  MotorPwm_SetEnabled(true);   /* EN always HIGH — same as ESP32. Stop = PWM 0. */
}

/* EN_MOTOR: active-high.  Invert if hardware is active-low. */
void MotorPwm_SetEnabled(bool enabled)
{
  HAL_GPIO_WritePin(EN_MOTOR_GPIO_Port, EN_MOTOR_Pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MotorPwm_StopAll(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void MotorPwm_Set(int motor_index_1based, int16_t pwm)
{
  switch (motor_index_1based)
  {
    case 1:
      if (CONTROL_INVERT_PWM1)
      {
        pwm = (int16_t)-pwm;
      }
      set_dual_pwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, pwm);
      return;

    case 2:
      if (CONTROL_INVERT_PWM2)
      {
        pwm = (int16_t)-pwm;
      }
      set_dual_pwm(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm);
      return;

    case 3:
      if (CONTROL_INVERT_PWM3)
      {
        pwm = (int16_t)-pwm;
      }
      set_dual_pwm(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, pwm);
      return;

    default:
      return;
  }
}
