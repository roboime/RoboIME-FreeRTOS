/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ID_Button_Pin GPIO_PIN_2
#define ID_Button_GPIO_Port GPIOE
#define ID_Button_EXTI_IRQn EXTI2_IRQn
#define M1_MBL_Pin GPIO_PIN_4
#define M1_MBL_GPIO_Port GPIOE
#define DAH_Pin GPIO_PIN_5
#define DAH_GPIO_Port GPIOE
#define M1_MAL_Pin GPIO_PIN_6
#define M1_MAL_GPIO_Port GPIOE
#define M0_MBL_Pin GPIO_PIN_13
#define M0_MBL_GPIO_Port GPIOC
#define BAT_REF_Pin GPIO_PIN_0
#define BAT_REF_GPIO_Port GPIOC
#define M0_CUR_Pin GPIO_PIN_1
#define M0_CUR_GPIO_Port GPIOC
#define M2_MAL_Pin GPIO_PIN_2
#define M2_MAL_GPIO_Port GPIOC
#define M2_ENCA_Pin GPIO_PIN_0
#define M2_ENCA_GPIO_Port GPIOA
#define M2_ENCB_Pin GPIO_PIN_1
#define M2_ENCB_GPIO_Port GPIOA
#define M1_CUR_Pin GPIO_PIN_2
#define M1_CUR_GPIO_Port GPIOA
#define M3_CUR_Pin GPIO_PIN_3
#define M3_CUR_GPIO_Port GPIOA
#define D_CUR_Pin GPIO_PIN_4
#define D_CUR_GPIO_Port GPIOA
#define M2_CUR_Pin GPIO_PIN_4
#define M2_CUR_GPIO_Port GPIOC
#define NRF_IRQ_Pin GPIO_PIN_5
#define NRF_IRQ_GPIO_Port GPIOC
#define CT_uc_Pin GPIO_PIN_0
#define CT_uc_GPIO_Port GPIOB
#define M2_MBL_Pin GPIO_PIN_1
#define M2_MBL_GPIO_Port GPIOB
#define INT_MPU_Pin GPIO_PIN_2
#define INT_MPU_GPIO_Port GPIOB
#define M2_MBH_Pin GPIO_PIN_11
#define M2_MBH_GPIO_Port GPIOE
#define M3_MBH_Pin GPIO_PIN_13
#define M3_MBH_GPIO_Port GPIOE
#define M3_MAH_Pin GPIO_PIN_14
#define M3_MAH_GPIO_Port GPIOE
#define M3_MBL_Pin GPIO_PIN_11
#define M3_MBL_GPIO_Port GPIOB
#define M3_MAL_Pin GPIO_PIN_12
#define M3_MAL_GPIO_Port GPIOB
#define CA_Pin GPIO_PIN_8
#define CA_GPIO_Port GPIOD
#define CB_Pin GPIO_PIN_10
#define CB_GPIO_Port GPIOD
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOD
#define LED_ORANGE_Pin GPIO_PIN_13
#define LED_ORANGE_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOD
#define M2_MAH_Pin GPIO_PIN_6
#define M2_MAH_GPIO_Port GPIOC
#define M0_MBH_Pin GPIO_PIN_7
#define M0_MBH_GPIO_Port GPIOC
#define M1_MBH_Pin GPIO_PIN_8
#define M1_MBH_GPIO_Port GPIOC
#define M0_MAH_Pin GPIO_PIN_9
#define M0_MAH_GPIO_Port GPIOC
#define M1_MAH_Pin GPIO_PIN_8
#define M1_MAH_GPIO_Port GPIOA
#define M1_ENCA_Pin GPIO_PIN_15
#define M1_ENCA_GPIO_Port GPIOA
#define MPU_CS_Pin GPIO_PIN_10
#define MPU_CS_GPIO_Port GPIOC
#define S2_Pin GPIO_PIN_11
#define S2_GPIO_Port GPIOC
#define NRF_CE_Pin GPIO_PIN_12
#define NRF_CE_GPIO_Port GPIOC
#define NRF_SS_Pin GPIO_PIN_0
#define NRF_SS_GPIO_Port GPIOD
#define S1_Pin GPIO_PIN_1
#define S1_GPIO_Port GPIOD
#define uSD_CS_Pin GPIO_PIN_3
#define uSD_CS_GPIO_Port GPIOD
#define Reset_Pin GPIO_PIN_4
#define Reset_GPIO_Port GPIOD
#define USB_overcurrent_Pin GPIO_PIN_5
#define USB_overcurrent_GPIO_Port GPIOD
#define M0_MAL_Pin GPIO_PIN_7
#define M0_MAL_GPIO_Port GPIOD
#define M1_ENCB_Pin GPIO_PIN_3
#define M1_ENCB_GPIO_Port GPIOB
#define M0_ENCA_Pin GPIO_PIN_4
#define M0_ENCA_GPIO_Port GPIOB
#define M0_ENCB_Pin GPIO_PIN_5
#define M0_ENCB_GPIO_Port GPIOB
#define M3_ENCA_Pin GPIO_PIN_6
#define M3_ENCA_GPIO_Port GPIOB
#define M3_ENCB_Pin GPIO_PIN_7
#define M3_ENCB_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
