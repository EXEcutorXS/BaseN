/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define BASE
#define USE_EEPROM

#define MAX_NODES 8
#define SAFE_INTERVAL 8
#define MAX_BASEID 9999999
#define WARNING_DELAY 650000
#define SUPER_WARNING_DELAY 1250000

	//Received message animation
#define FASTBLINK_ANIMATION // WAVE_ANIMATION or FASTBLINK_ANIMATION

#define BAD_INTERVAL -1

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sx127x.h"
#include "uartBase.h"
#include "ws2812.h"
#include "otisProtocol.h"
#include "common.h"
#include "SSD1306buf.h"
#include "math.h"
#include "Network.h"
#include <stdbool.h>
#include "UC1609.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct flag_s
{
uint32_t encInc:1;
uint32_t encDec:1;
uint32_t encOk:1;
uint32_t back:1;
uint32_t saveSettings:1;
uint32_t sendConfig:1;
uint32_t lcdRefreshRequest:1;
uint32_t checkConfig:1;
uint32_t uartGotMessage:1;
}flag_t;



typedef  enum screenModes{
	smRegular,
	smMainMenu,
	smRadioMenu,
	smBaseMenu,
	smNodeMenu,
	smNetworkMenu,
	smFrequency,
	smSf,
	smBw,
	smSw,
	smCr,
	smPreamble,
	smPower,
	smInterval1,
	smInterval2,
	smWorkInterval,
	smLed,
	smCleanNodeData,
	smRefreshNetworks,
	smSelectNet,
	smSetWiFiPassword,
	smSetServerPassword,
	smSetHost,
	smSetBaseID,
	smSaveConfig,
	smStatus,
	smFirmware,
	smMessageCounter,
	smNodeAction,
	smModeSelect

} screen_t;

typedef enum netStatus
{
netStatusUninitialised,  //Первое включение
netStatusInitialised,    //Заданы параметры (SSID, пароль, IP)
netStatusConnected,      //Есть подключение к точке доступа
netStatusOnline,	       //Есть выход в интернет
netStatusOffline,        //Есть подключение но нет выхода в интернет
netStatusBusy	           //Передаётся сообщение
}netStatus_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RESET_Pin GPIO_PIN_3
#define RESET_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_1
#define B1_GPIO_Port GPIOB
#define B1_EXTI_IRQn EXTI1_IRQn
#define B2_Pin GPIO_PIN_2
#define B2_GPIO_Port GPIOB
#define B2_EXTI_IRQn EXTI2_IRQn
#define B3_Pin GPIO_PIN_10
#define B3_GPIO_Port GPIOB
#define B3_EXTI_IRQn EXTI15_10_IRQn
#define B4_Pin GPIO_PIN_11
#define B4_GPIO_Port GPIOB
#define B4_EXTI_IRQn EXTI15_10_IRQn
#define CS_Pin GPIO_PIN_12
#define CS_GPIO_Port GPIOB
#define CD_Pin GPIO_PIN_14
#define CD_GPIO_Port GPIOB
#define RELAY_Pin GPIO_PIN_8
#define RELAY_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


void alarmOff(void);
void ledActivity();
void encoderPinChanged();
void saveSettings();
void button_right();
void button_left();
void button_ok();
void button_back();
void lcdActivity();
void changeDisarmNode(int16_t selectedNode);
void saveNodeData();
uint8_t tryEeprom();
uint32_t secondsInMonth(uint8_t month, uint8_t year);
uint8_t checkFreq(uint32_t frequency, uint8_t bw);

//Тесты
void colorTest();
void radioTestTransmit();
void radioTestReceive();
void radioTestPing();
void radioSignalIndicator();


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
