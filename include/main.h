/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LINE_D1_Pin GPIO_PIN_0
#define LINE_D1_GPIO_Port GPIOC
#define LINE_D2_Pin GPIO_PIN_1
#define LINE_D2_GPIO_Port GPIOC
#define TRIG_DEPAN_Pin GPIO_PIN_2
#define TRIG_DEPAN_GPIO_Port GPIOC
#define TRIG_KANAN_Pin GPIO_PIN_3
#define TRIG_KANAN_GPIO_Port GPIOC
#define M3_RPWM_Pin GPIO_PIN_0
#define M3_RPWM_GPIO_Port GPIOA
#define M3_LPWM_Pin GPIO_PIN_1
#define M3_LPWM_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define EN_STEPPER_Pin GPIO_PIN_5
#define EN_STEPPER_GPIO_Port GPIOA
#define ENC1_CH_A_Pin GPIO_PIN_6
#define ENC1_CH_A_GPIO_Port GPIOA
#define ENC1_CH_B_Pin GPIO_PIN_7
#define ENC1_CH_B_GPIO_Port GPIOA
#define ECHO5_Pin GPIO_PIN_4
#define ECHO5_GPIO_Port GPIOC
#define ECHO6_Pin GPIO_PIN_5
#define ECHO6_GPIO_Port GPIOC
#define EN_MOTOR_Pin GPIO_PIN_0
#define EN_MOTOR_GPIO_Port GPIOB
#define ECHO8_Pin GPIO_PIN_2
#define ECHO8_GPIO_Port GPIOB
#define STP1_Pin GPIO_PIN_10
#define STP1_GPIO_Port GPIOB
#define ECHO7_Pin GPIO_PIN_12
#define ECHO7_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_14
#define SERVO2_GPIO_Port GPIOB
#define SERVO3_Pin GPIO_PIN_15
#define SERVO3_GPIO_Port GPIOB
#define ENC3_CH_A_Pin GPIO_PIN_6
#define ENC3_CH_A_GPIO_Port GPIOC
#define ENC3_CH_B_Pin GPIO_PIN_7
#define ENC3_CH_B_GPIO_Port GPIOC
#define ECHO1_Pin GPIO_PIN_8
#define ECHO1_GPIO_Port GPIOC
#define ECHO2_Pin GPIO_PIN_9
#define ECHO2_GPIO_Port GPIOC
#define M1_RPWM_Pin GPIO_PIN_8
#define M1_RPWM_GPIO_Port GPIOA
#define M1_LPWM_Pin GPIO_PIN_9
#define M1_LPWM_GPIO_Port GPIOA
#define M2_RPWM_Pin GPIO_PIN_10
#define M2_RPWM_GPIO_Port GPIOA
#define M2_LPWM_Pin GPIO_PIN_11
#define M2_LPWM_GPIO_Port GPIOA
#define DIR1_Pin GPIO_PIN_12
#define DIR1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DIR2_Pin GPIO_PIN_15
#define DIR2_GPIO_Port GPIOA
#define ECHO3_Pin GPIO_PIN_10
#define ECHO3_GPIO_Port GPIOC
#define ECHO4_Pin GPIO_PIN_11
#define ECHO4_GPIO_Port GPIOC
#define TRIG_KIRI_Pin GPIO_PIN_12
#define TRIG_KIRI_GPIO_Port GPIOC
#define TRIG_BELAKANG_Pin GPIO_PIN_2
#define TRIG_BELAKANG_GPIO_Port GPIOD
#define DIR3_Pin GPIO_PIN_3
#define DIR3_GPIO_Port GPIOB
#define STP2_Pin GPIO_PIN_4
#define STP2_GPIO_Port GPIOB
#define STP3_Pin GPIO_PIN_5
#define STP3_GPIO_Port GPIOB
#define ENC2_CH_A_Pin GPIO_PIN_6
#define ENC2_CH_A_GPIO_Port GPIOB
#define ENC2_CH_B_Pin GPIO_PIN_7
#define ENC2_CH_B_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_8
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_9
#define MPU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
