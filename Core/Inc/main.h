/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
	
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

typedef struct PID{
	volatile double SetPoint;            //设定值
	double Kp;                  //比例系数
	double Ki;                  //积分系数
	double Kd;                  //微分系数
	double LastError;           //最后一次误差数Er[-1]
	double PrevError;           //最后第二次误差数er[-2]
	double SumError;            //误差积分  
}PID;


extern PID wheel_speed_1;
extern PID wheel_speed_2;
extern PID wheel_speed_3;
extern PID wheel_speed_4;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WHEEL3_DIR_Pin GPIO_PIN_6
#define WHEEL3_DIR_GPIO_Port GPIOE
#define WHEEL4_DIR_Pin GPIO_PIN_6
#define WHEEL4_DIR_GPIO_Port GPIOF
#define BEEP_Pin GPIO_PIN_1
#define BEEP_GPIO_Port GPIOC
#define IPS114_BL_Pin GPIO_PIN_13
#define IPS114_BL_GPIO_Port GPIOE
#define IPS114_CS_Pin GPIO_PIN_14
#define IPS114_CS_GPIO_Port GPIOE
#define IPS114_DC_Pin GPIO_PIN_15
#define IPS114_DC_GPIO_Port GPIOE
#define IPS114_REST_Pin GPIO_PIN_8
#define IPS114_REST_GPIO_Port GPIOD
#define IPS114_SDA_Pin GPIO_PIN_9
#define IPS114_SDA_GPIO_Port GPIOD
#define IPS114_SCL_Pin GPIO_PIN_10
#define IPS114_SCL_GPIO_Port GPIOD
#define WHEEL1_DIR_Pin GPIO_PIN_7
#define WHEEL1_DIR_GPIO_Port GPIOC
#define WHEEL2_DIR_Pin GPIO_PIN_9
#define WHEEL2_DIR_GPIO_Port GPIOC
#define RTS_Pin GPIO_PIN_3
#define RTS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
