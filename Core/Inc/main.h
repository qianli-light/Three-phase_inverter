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
#include "stm32g4xx_hal.h"

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
  typedef struct rotation_vector {
    float w0;
    float t0;
    float theta;
    float vm;
  }SV;
  typedef struct PR {
    float w0;
    float t;
    float k;
    float wc;

    float a1;
    float a2;
    float b0;
    float b2;

    float kp;
    float kr;

    float e1;
    float e2;
    float y0;
    float y1;
    float y2;

    float u0;
  }QPR;
  typedef struct SVPWM {
    float vdc;
    float Ts;
    float T0;
    float T1;
    float T2;
    float arr;
    float mid;
  }SVPWM;
  enum EC_DeBug{
    vm_=0,
    kp1_=1,
    kr1_=2,
    kp2_=3,
    kr2_=4,
  };
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
  void SV_init(SV *S,float w0,float t0,float vm);
  void QPR_Init(QPR *QPR,float wc,float t,float w0,float kp,float kr);
  void SVPWM_init(SVPWM *SVPWM,float vdc,float Ts,float arr);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EC11D_Pin GPIO_PIN_13
#define EC11D_GPIO_Port GPIOC
#define EC11D_EXTI_IRQn EXTI15_10_IRQn
#define EC11B_Pin GPIO_PIN_14
#define EC11B_GPIO_Port GPIOC
#define EC11B_EXTI_IRQn EXTI15_10_IRQn
#define EC11A_Pin GPIO_PIN_15
#define EC11A_GPIO_Port GPIOC
#define EC11A_EXTI_IRQn EXTI15_10_IRQn
#define KEY2_Pin GPIO_PIN_0
#define KEY2_GPIO_Port GPIOB
#define KEY2_EXTI_IRQn EXTI0_IRQn
#define KEY0_Pin GPIO_PIN_12
#define KEY0_GPIO_Port GPIOA
#define KEY0_EXTI_IRQn EXTI15_10_IRQn
#define OLED_MOSI_Pin GPIO_PIN_12
#define OLED_MOSI_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOD
#define KEY1_EXTI_IRQn EXTI2_IRQn
#define OLED_SCK_Pin GPIO_PIN_3
#define OLED_SCK_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_5
#define OLED_DC_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_9
#define OLED_RES_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
