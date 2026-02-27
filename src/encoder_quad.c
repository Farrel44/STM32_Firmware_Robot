#include "encoder_quad.h"

#include "stm32f4xx_hal_tim.h"

void EncoderQuad_ConfigureFullQuad(TIM_HandleTypeDef *htim)
{
  /*
   * ESP32 code uses full quadrature counting. For STM32 timers, that means:
   * - Encoder mode TI12 (both channels participate)
   * - Both-edge polarity on both inputs (4x decoding)
   */
  TIM_Encoder_InitTypeDef cfg = {0};
  cfg.EncoderMode = TIM_ENCODERMODE_TI12;

  cfg.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
  cfg.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  cfg.IC1Prescaler = TIM_ICPSC_DIV1;
  cfg.IC1Filter = 0;

  cfg.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;
  cfg.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  cfg.IC2Prescaler = TIM_ICPSC_DIV1;
  cfg.IC2Filter = 0;

  (void)HAL_TIM_Encoder_Init(htim, &cfg);
}
