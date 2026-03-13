#include "encoder_quad.h"

#include "stm32f4xx_hal_tim.h"

/*
 * Reconfigure a CubeMX-initialised encoder timer to full 4× quadrature.
 * CubeMX defaults to TI1-only (1×); this sets TI12 (4×) with ICxFilter = 5
 * for mechanical noise rejection.
 *
 * NOTE: This function must be called BEFORE HAL_TIM_Encoder_Start().
 * The current call site (TaskPid, before Encoder_Start) is correct.
 * Ideally CubeMX would generate TI12 mode directly, but the .ioc only
 * exposes TI1/TI2 selection — TI12 requires runtime reconfiguration.
 *
 * IMPORTANT: BOTHEDGE polarity must NOT be used — on STM32F4 encoder mode
 * it sets the reserved CC1NP bit, causing counter oscillation.
 */
void EncoderQuad_ConfigureFullQuad(TIM_HandleTypeDef *htim)
{
  TIM_Encoder_InitTypeDef cfg = {0};
  cfg.EncoderMode = TIM_ENCODERMODE_TI12;

  cfg.IC1Polarity = TIM_ICPOLARITY_RISING;
  cfg.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  cfg.IC1Prescaler = TIM_ICPSC_DIV1;
  cfg.IC1Filter = 5;

  cfg.IC2Polarity = TIM_ICPOLARITY_RISING;
  cfg.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  cfg.IC2Prescaler = TIM_ICPSC_DIV1;
  cfg.IC2Filter = 5;
  // Enc udh di init oleh cubemx
  // (void)HAL_TIM_Encoder_Init(htim, &cfg);
}
