/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_freertos.h
  * @brief   CMSIS-RTOS2 (FreeRTOS) application layer.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Thread handles exposed for ISR notifications. */
extern osThreadId_t pidTaskHandle;

/* Thread flags */
#define PID_TICK_FLAG   (1UL << 0)

void MX_FREERTOS_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_FREERTOS_H */
