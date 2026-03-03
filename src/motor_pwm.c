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

/* Start all PWM channels at 0 % duty before enabling the driver. */
void MotorPwm_Init(void)
{
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  MotorPwm_StopAll();
  MotorPwm_SetEnabled(false);
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
