#ifndef ENCODER_QUAD_H
#define ENCODER_QUAD_H

#include "stm32f4xx_hal.h"

/*
 * Ensure encoder timers count full quadrature (4x) to match the tick definition
 * used by the original ESP implementation (TICKS_PER_REV).
 *
 * Why this exists:
 * - CubeMX settings may be regenerated; keeping this in application code makes the
 *   control loop behavior explicit and reviewable.
 */
void EncoderQuad_ConfigureFullQuad(TIM_HandleTypeDef *htim);

#endif /* ENCODER_QUAD_H */
