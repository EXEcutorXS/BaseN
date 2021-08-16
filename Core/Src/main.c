/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

extern Logo[];
const uint32_t version = 0x08082021;

const uint32_t bandWidth[10] = { 7800, 10400, 15600, 20800, 31200, 41700, 62500, 125000, 250000, 500000 };

const uint8_t mW[] = { 10, 13, 16, 20, 25, 32, 40, 50, 63, 80, 100 };

const char *maskNames[] = { "Full", "Only power", "Only door", "None" };
const char *bandwidthNames[] = { "7.8 kHz", "10.4 kHz", "15.6 kHz", "20.8 kHz", "31.2 kHz", "42.6 kHz", "62.5 kHz", "125 kHz", "250 kHz", "500 kHz" };
flag_t flag;

char string[8][64] = { 0, };

SX127X_t myRadio;

uint32_t receivedMesCnt = 0;
uint32_t receivedMesCntSuc = 0;
uint8_t lastMessageFrom = 255;
uplinkMessage_t *rxMes = (uplinkMessage_t*) myRadio.rxBuf;
downlinkMessage_t *txMes = (downlinkMessage_t*) myRadio.txBuf;
nodeHandler_t nodes[MAX_NODES];
nodeDataToSave_t nodeData[MAX_NODES];

baseSettings_t settings;

nodeSettings_t nodeSettings;

uint32_t configTime;

screen_t screenMode = smRegular;
uint8_t idleMode = 0;
int8_t menuPosition = 0;
selectedMode_t selectedMode = work;

NetHandler_t netHandler;

int16_t selectedNode = 0;
uint32_t lastNodeChangeTick = 0;

uint8_t uartIn;
uint32_t lastUartConnect;

uint16_t menuDelayCounter = 0;

availableFrequences_t legalFreq;
int8_t currentInterval;

uint8_t configStep; //Current step of node configure

uint8_t data;

int8_t selectedNetwork = 0;
int8_t cursorPos = 0;

bool blinkProvider;
bool fastBlinkProvider;

uint32_t upTime;
uint32_t transmittingTime;
uint32_t receivingTime;
float airUseForTx;
float airUseForRx;

uint32_t absoluteMaxDelay = 0;

uint16_t adc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write (int fd, char *ptr, int len)
{
	HAL_UART_Transmit (&huart1, (uint8_t*) ptr, len, 1000);
	return len;
}

int __io_putchar (int ch)
{
	HAL_UART_Transmit (&huart1, (uint8_t*) &ch, 1, 100);
	return ch;
}

void eraseNodeData ()
{
	for (int i = 0; i < MAX_NODES; ++i)
		{
			nodeData[i].disarmRequest = 0;
			nodeData[i].disarmed = 0;
			nodeData[i].masked = 0;
		}
	writeToFram (0, (uint8_t*) &nodeData, sizeof(nodeData), &hi2c1);
}

void saveNodeData ()
{
	for (int i = 0; i < MAX_NODES; ++i)
		{
			nodeData[i].disarmRequest = nodes[i].disarmRequest;
			nodeData[i].disarmed = nodes[i].disarmed;
			nodeData[i].masked = nodes[i].masked;
		}
	writeToFram (0, (uint8_t*) &nodeData, sizeof(nodeData), &hi2c1);
}

void loadNodeData ()
{
	readFromFram ((uint8_t*) &nodeData, 0, sizeof(nodeData), &hi2c1);
	for (int i = 0; i < MAX_NODES; ++i)
		{
			nodes[i].disarmRequest = nodeData[i].disarmRequest;
			nodes[i].disarmed = nodeData[i].disarmed;
			nodes[i].masked = nodeData[i].masked;
		}

}
/**
 * @brief Callback for all EXTI lines
 * @param  GPIO_Pin - changed pin
 * @retval None
 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case GPIO_PIN_1:
			delayMicro (20000);
			if (HAL_GPIO_ReadPin (B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
				flag.encOk = 1;
			break;

		case GPIO_PIN_2:
			delayMicro (20000);
			if (HAL_GPIO_ReadPin (B2_GPIO_Port, B2_Pin) == GPIO_PIN_RESET)
				flag.back = 1;
			break;

		case GPIO_PIN_10:
		case GPIO_PIN_11:
			encoderPinChanged ();
			break;
	}
}

/**
 * @brief Callback for UART interrupt
 * @param  *huart - uart handler
 * @retval None
 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT (&huart1, &uartIn, 1);
	lastUartConnect = HAL_GetTick ();
	readByte ();
}

/**
 * @brief sets settings to default values
 * @param None
 * @retval None
 */
void defaultSettings ()
{
	settings.bw = SX127X_LORA_BW_125KHZ;
	settings.cr = SX127X_CR_4_8;
	settings.realFrequency = DEF_FREQUENCY;
	settings.power = SX127X_POWER_20DBM;
	settings.sf = SX127X_LORA_SF_12;
	settings.preamble = 5;
	settings.superWarningDelay = SUPER_WARNING_DELAY;
	settings.syncWord = 0x1;
	settings.warningDelay = WARNING_DELAY;
	settings.realFrequency = DEF_FREQUENCY;
	settings.baseID = 1;
	nodeSettings.workInterval = 600;
	nodeSettings.useLed = 1;
	sprintf (settings.SSID, "EXEcutor");
	sprintf (settings.WiFiPass, "executor");
	sprintf (settings.ServerPass, "12345");
	sprintf (settings.ServerPass, "12345");

}

/**
 * @brief Fills all strings with '\0' char
 * @param None
 * @retval None
 */
void clearStrings ()
{
	memset (string, 0, sizeof(string));
}

void setCursor (char *string, int8_t pos)
{
	for (int i = 0; i < pos; i++)
		string[i] = ' ';
	string[pos] = '^';
}

/**
 * @brief Writes string[0]..string[3] to all screens
 * @param None
 * @retval None
 */
void updateLcd ()
{
	ssd1306_Clean ();
	for (int i = 0; i < 8; i++)
		{
			ssd1306_GotoXY (0, i);
			ssd1306_PutString (string[i]);
		}

	ssd1306_Update ();

	 UC1609_Clean ();

	 for (int i = 0; i < 8; i++)
	 {
	 UC1609_SetPos (0, i);
	 UC1609_PutString (string[i]);
	 }
	 UC1609_UpdateScreen ();

}

/**
 * @brief Initiates settings from settings structure to radio structure
 * @param None
 * @retval None
 */
void settingsInitiate ()
{
	myRadio.sf = settings.sf;
	myRadio.bw = settings.bw;
	myRadio.cr = settings.cr;
	myRadio.frequency = (uint32_t) (settings.realFrequency / 61.035f);
	myRadio.preamble = settings.preamble;
	myRadio.power = settings.power;
	myRadio.syncWord = settings.syncWord;
	myRadio.preamble = settings.preamble;
}

_Bool isBaseSettingsValid (baseSettings_t *s)
{
	if (s->realFrequency >= MIN_FREQUENCY && s->realFrequency <= MAX_FREQUENCY)
		if (s->sf > 6 && s->sf < 13)
			if (s->bw < 10)
				if (s->cr >= 1 && s->cr <= 4)
					if (s->preamble > 1 && s->preamble < 65535)
						if (s->realFrequency > MIN_FREQUENCY && s->realFrequency < MAX_FREQUENCY)
							if (s->syncWord != 0x34 && s->syncWord != 0x55 && s->syncWord != 0xAA)
								if (s->power <= 20 || s->power >= 10)
									if (s->warningDelay > MIN_WARNING_DELAY && s->warningDelay < MAX_WARNING_DELAY)
										if (s->superWarningDelay > MIN_WARNING_DELAY && s->superWarningDelay < MAX_WARNING_DELAY)
											if (s->superWarningDelay > s->warningDelay)
												if (s->preamble >= MIN_PREAMBLE && s->preamble <= MAX_PREAMBLE)
													if (s->baseID > 0 && s->baseID < MAX_BASEID)
														return true;

	return false;
}

_Bool isNodeSettingsValid (nodeSettings_t *s)
{
	if (s->workInterval >= MIN_WORK_INTERVAL && s->workInterval <= MAX_WORK_INTERVAL)
		return true;

	return false;
}

/**
 * @brief tries to load settings from page 63 flash memory
 * @param None
 * @retval 1 - success 0 - fail
 */
bool tryLoadSettings ()
{
	bool ret = true;
	baseSettings_t loadedBaseSettings;
	nodeSettings_t loadedNodeSettings;
	readFromFram ((uint8_t*) &loadedBaseSettings, BASE_SETTINGS_SHIFT, sizeof(baseSettings_t), &hi2c1);
	readFromFram ((uint8_t*) &loadedNodeSettings, NODE_SETTINGS_SHIFT, sizeof(nodeSettings_t), &hi2c1);
	if (isBaseSettingsValid (&loadedBaseSettings))
		settings = loadedBaseSettings;
	else
		ret = 0;
	if (isNodeSettingsValid (&loadedNodeSettings))
		nodeSettings = loadedNodeSettings;
	else
		ret = 0;
	settingsInitiate ();
	return ret;
}

/**
 * @brief Saves current device settings from RAM to page 63 of internal flash
 * @param None
 * @retval None
 */
void saveSettings ()
{
	writeToFram (BASE_SETTINGS_SHIFT, (uint8_t*) &settings, sizeof(settings), &hi2c1);
	writeToFram (NODE_SETTINGS_SHIFT, (uint8_t*) &nodeSettings, sizeof(nodeSettings), &hi2c1);
	settingsInitiate ();
}

/**
 * @brief Encoder handler. Must be called when encoder pin status is changed
 * @param None
 * @retval None
 */
void encoderPinChanged ()
{
	static uint8_t right;
	static uint8_t left;
	static uint16_t lastStatus;
	static uint8_t r = 0;
	static uint8_t l = 0;
	uint32_t status = *(uint32_t*) 0x40010C08 & 0xC00;

	switch (status)
	{
		case (0xC00):
			if (right > 2)
				flag.encDec = 1;
			if (left > 2)
				flag.encInc = 1;
			right = 0;
			left = 0;
			r = 0;
			l = 0;
			break;

		case (0x400):
			if (lastStatus == 0xC00)
				{
					right++;
					r = 1;
					l = 0;
				}

			if (lastStatus == 0 && l)
				left++;
			break;

		case 0:
			if (lastStatus == 0x400 && r)
				right++;

			if (lastStatus == 0x800 && l)
				left++;
			break;

		case (0x800):

			if ((lastStatus == 0x000) && r)
				right++;

			if (lastStatus == 0xC00)
				{
					left++;
					l = 1;
					r = 0;
				}
	}
	lastStatus = status;
}

uint8_t nextNodeAwaited ()
{
	uint8_t ret = 255;
	int32_t minDelay = 0x40000000;
	uint32_t tick = HAL_GetTick ();
	for (int i = 0; i < MAX_NODES; i++)
		{
			if (nodes[i].activated && ((long) nodes[i].NextMessageTick - (long) tick) < minDelay)
				{
					ret = i;
					minDelay = nodes[i].NextMessageTick - tick;
				}
		}
	return ret;
}

void calcDelay (uint8_t nodeNum)
{
	uint32_t maxDelay = 0;

	uint16_t delay = 0;

	uint32_t tick = HAL_GetTick ();

	for (int i = 0; i < MAX_NODES; i++)
		{
			if (nodes[i].activated && ((nodes[i].NextMessageTick - tick) > (maxDelay * 1000)) && (nodes[i].NextMessageTick - tick < 0x8000000)
					&& (i != nodeNum))
				maxDelay = (nodes[i].NextMessageTick - tick) / 1000;
		}

	if (maxDelay > (nodeSettings.workInterval - SAFE_INTERVAL))
		delay = maxDelay + SAFE_INTERVAL;
	else
		delay = nodeSettings.workInterval;
	nodes[nodeNum].NextMessageTick = tick + delay * 1000;
	nodes[nodeNum].delay = delay;
	absoluteMaxDelay = absoluteMaxDelay < delay ? delay : absoluteMaxDelay;

}
/**
 * @brief Received radio message handler
 * @param message length
 * @retval None
 */
void handleMessage (uint8_t len)
{
	uint8_t nodeNum = rxMes->adr;
	receivedMesCnt++;
	clearStrings ();
	if (rxMes->uplink == 0)
		{
			printf ("Received downlink message!\n");
			return;
		}
	if (myRadio.badCrc == 1)
		{
			printf ("Bad CRC in received message!\n");
			return;
		}

	if (rxMes->adr >= MAX_NODES)
		{
			printf ("Node %d not supported! Max is %d\n", nodeNum, MAX_NODES);
			return;
		}

	receivedMesCntSuc++;

	if (nodes[nodeNum].activated == 0)
		{
			nodes[nodeNum].activated = 1;
			saveNodeData ();
		}
	nodes[nodeNum].disarmed = rxMes->disarm;
	nodes[nodeNum].lastContact = HAL_GetTick ();
	nodes[nodeNum].opened = rxMes->opened;
	nodes[nodeNum].powered = rxMes->powered;
	nodes[nodeNum].voltage = 1.9F + rxMes->codedVoltage / 10.0F;
	nodes[nodeNum].temperature = rxMes->codedTemperature / 2.0F - 40.0F;
	nodes[nodeNum].rssi = SX127X_RSSI_Pack (&myRadio);
	if ((nodes[nodeNum].masked & MASK_DOOR) == 0 && nodes[nodeNum].opened)
		{
			nodes[nodeNum].alarm |= ALARM_DOOR;
			nodes[nodeNum].NetAlarm |= ALARM_DOOR;
		}
	if ((nodes[nodeNum].masked & MASK_POWER) == 0 && !nodes[nodeNum].powered)
		{
			nodes[nodeNum].alarm |= ALARM_POWER;
			nodes[nodeNum].NetAlarm |= ALARM_POWER;
		}

	lastMessageFrom = nodeNum;
	flag.lcdRefreshRequest = 1;
	calcDelay (nodeNum);
	txMes->codedDelayLSB = nodes[nodeNum].delay & 0xFF;
	txMes->codedDelayMSB = (nodes[nodeNum].delay >> 8) & 0x7F;
	txMes->adr = nodeNum;
	txMes->disarm = nodes[nodeNum].disarmRequest;
	txMes->message = MSG_DOWN_ACKNOWLEDGE;
	txMes->uplink = 0;
	HAL_Delay (2);
	SX127X_transmitAsync (&myRadio, 3);

	if (netHandler.online)
		{
			char message[256];
			char Vstr[16];
			char Tstr[16];
			char DAstr[16];
			char PAstr[16];
			sprintf (DAstr, "%s", nodes[nodeNum].NetAlarm & ALARM_DOOR ? "&DA=true" : "");
			sprintf (PAstr, "%s", nodes[nodeNum].NetAlarm & ALARM_POWER ? "&PA=true" : "");
			sprintf (Vstr, "&V=%d.%d", (19 + rxMes->codedVoltage) / 10, (19 + rxMes->codedVoltage) % 10);
			sprintf (Tstr, "&T=%d.%d", (int) nodes[nodeNum].temperature, ((int) (nodes[nodeNum].temperature * 10.0f)) % 10);
			sprintf (message, "%s/Uplink/UpdateNode?N=%d&BID=%lu&Ps=%s&O=%s&P=%s&D=%s&DM=%s&PM=%s%s%s%s%s", settings.host, rxMes->adr, settings.baseID,
								settings.ServerPass, rxMes->opened ? "true" : "false", rxMes->powered ? "true" : "false", rxMes->disarm ? "true" : "false",
								nodes[nodeNum].masked & MASK_DOOR ? "true" : "false", nodes[nodeNum].masked & MASK_POWER ? "true" : "false", Vstr, Tstr, DAstr,
								PAstr);

			NetSendAsync (&netHandler, message);
			memset (message, 0, sizeof(message));
			nodes[nodeNum].NetAlarm = ALARM_OFF;
		}

	memset (myRadio.rxBuf, 0, sizeof(myRadio.rxBuf));

}

void analogRoutine ()
{
	static _Bool pressed[4];
	static uint16_t counters[4] = { 0, };
	uint16_t adc = HAL_ADC_GetValue (&hadc2);

	if (adc < 500)
		{

			for (uint8_t i = 0; i < 4; ++i)
				if (counters[i])
					counters[i]--;
			for (uint8_t i = 0; i < 4; ++i)
				if (pressed[i] && !counters[i])
					pressed[i] = 0;

		}
	if (adc < 950 && adc > 600 && pressed[3] == 0)
		{
			counters[3]++;
			if (counters[3] > 100)
				{
					pressed[3] = true;
					flag.analog3 = 1;
				}
		}
	if (adc < 1100 && adc > 950 && pressed[2] == 0)
		{
			counters[2]++;
			if (counters[2] > 100)
				{
					pressed[2] = true;
					flag.analog2 = 1;
				}
		}

	if (adc < 1500 && adc > 1200 && pressed[1] == 0)
		{
			counters[1]++;
			if (counters[1] > 100)
				{
					pressed[1] = true;
					flag.analog1 = 1;
				}
		}
	if (adc > 1500 && pressed[0] == 0)
		{
			counters[0]++;
			if (counters[0] > 100 && pressed[0] == 0)
				{
					pressed[0] = true;
					flag.analog0 = 1;
				}
		}

}
/**
 * @brief Handles alarm situations
 * @param None
 * @retval None
 */
void alarmRoutine ()
{
	uint16_t i;
	uint8_t gotAlarm = 0;
	for (i = 0; i < MAX_NODES; i++)
		if (nodes[i].alarm)
			gotAlarm = true;

	if (gotAlarm)
		{
			uint8_t tact = (HAL_GetTick () / 80) % 12;
			if (tact % 2 && tact < 9)
				HAL_GPIO_WritePin (BUZZER_GPIO_Port, BUZZER_Pin, 1);
			else
				HAL_GPIO_WritePin (BUZZER_GPIO_Port, BUZZER_Pin, 0);
			HAL_GPIO_WritePin (RELAY_GPIO_Port, RELAY_Pin, 0);
		}
	else
		{
			HAL_GPIO_WritePin (BUZZER_GPIO_Port, BUZZER_Pin, 0);
			HAL_GPIO_WritePin (RELAY_GPIO_Port, RELAY_Pin, 1);
		}
}

/**
 * @brief Turns alarm indication off and alarm flag for every node
 * @param None
 * @retval None
 */
void alarmOff ()
{
	uint16_t i;
	for (i = 0; i < MAX_NODES; i++)
		nodes[i].alarm = 0;
}

void ledNodeStatusIndication ()
{
	int i;
	for (i = 0; i < MAX_NODES; i++)
		{

			wsSetColor (i + 1, GREEN);
			if (nodes[i].powered == 0)
				wsSetColor (i + 1, CYAN);

			if (nodes[i].voltage < nodes[i].voltageTrashold)
				blinkProvider ? wsSetColor (i + 1, CYAN) : wsSetColor (i + 1, BLACK);

			if (nodes[i].masked & MASK_DOOR)
				(nodes[i].opened && blinkProvider) ? wsSetColor (i + 1, BLACK) : wsSetColor (i + 1, BLUE); // Blinking - opened

			if (nodes[i].alarm)
				(blinkProvider && nodes[i].alarm == ALARM_DOOR) ? wsSetColor (i + 1, BLACK) : wsSetColor (i + 1, RED); //Blinking - DOOR ALARM still - POWER ALARM

			if (HAL_GetTick () - nodes[i].lastContact > settings.warningDelay && HAL_GetTick () > (nodes[i].NextMessageTick + 10000))
				wsSetColor (i + 1, YELLOW);
			if (HAL_GetTick () - nodes[i].lastContact > settings.superWarningDelay && blinkProvider && HAL_GetTick () > (nodes[i].NextMessageTick + 10000))
				wsSetColor (i + 1, BLACK);

			if (nodes[i].disarmed && nodes[i].disarmRequest)
				wsSetColor (i + 1, ORANGE);

			if (nodes[i].disarmRequest != nodes[i].disarmed)
				wsSetColor (i + 1, MAGENTA);

			if (nodes[i].activated == 0)
				wsSetColor (i + 1, WHITE);

			if (selectedNode == i && HAL_GetTick () - lastNodeChangeTick < DELAY_SELECTED_MARK && lastNodeChangeTick)
				wsDoubleBright (i + 1);

			if (HAL_GetTick () - nodes[i].lastContact < DELAY_SHOW_CONTACT && nodes[i].activated)
				{
#if RECEIVE_ANIMATION == WAVE
					float fTemp;
					uint32_t temp = (HAL_GetTick () - nodes[i].lastContact) % 1000;
					if (temp < 500)
						fTemp = 1.0f + (float) temp / 250.0f;
					else
						fTemp = 5.0f - (float) temp / 250.0f;

					wsMultiply (i + 1, fTemp);
#else
					if (fastBlinkProvider)
						wsDoubleBright (i + 1);
#endif
				}
		}
}

void ledFillBlack ()
{
	int i;
	for (i = 0; i < MAX_NODES + 1; i++)
		wsSetColor (i, BLACK);
}
/**
 * @brief Handles WS2812 LEDs
 * @param None
 * @retval None
 */
void ledRoutine ()
{
	uint8_t i = 0;

	wsSetColor (0, BLACK);

	if (myRadio.status == TX)
		wsSetColor (0, PINK);
	if (myRadio.signalDetected == 1)
		wsSetColor (0, LGREEN);
	if (HAL_GetTick () - lastUartConnect < 200)
		wsSetColor (0, YELLOW);

	if (screenMode == smRegular || screenMode == smGraph)
		{
			ledNodeStatusIndication ();
		}
	else if (screenMode == smSaveConfig)
		{
			ledFillBlack ();
		}
	else if (screenMode == smNodeAction)
		{
			for (i = 0; i < MAX_NODES + 1; i++)
				{
					wsSetColor (i, BLACK);
				}
			wsSetColor (selectedNode + 1, WHITE);
		}
	else
		{
			for (i = 0; i < MAX_NODES + 1; i++)
				{
					wsSetColor (i, BLACK);
				}

		}
	wsPrepareArray ();
}

/**
 * @brief Handles turn on logo
 * @param None
 * @retval None
 */
void ShowLogo ()
{
	int i = 0;
	UC1609_DrawBitmap(Logo);
	UC1609_UpdateScreen();
	HAL_Delay(1000);
	for (i = 0; i < MAX_NODES + 1; i++)
		{
			wsSetColor (i, WHITE);
			wsPrepareArray ();
			HAL_Delay (20);
		}
	clearStrings ();
	sprintf (string[0], "SX127* is OK ");
	sprintf (string[1], "Module Firmware:");
	sprintf (string[2], "0x%X", myRadio.revision);
	updateLcd ();
	HAL_Delay (300);
}

/**
 * @brief Arms/Disarms selected node
 * @param  selectedNode obvious
 * @retval None
 */
void changeDisarmNode (int16_t selectedNode)
{
	if (nodes[selectedNode].powered)
		{
			nodes[selectedNode].disarmRequest = !nodes[selectedNode].disarmRequest;
			txMes->adr = selectedNode;
			txMes->disarm = nodes[selectedNode].disarmRequest;
			txMes->message = MSG_DOWN_REQUEST;
			txMes->uplink = 0;
			SX127X_transmitAsync (&myRadio, 3);
		}
	else
		nodes[selectedNode].disarmRequest = !nodes[selectedNode].disarmRequest;

}

/**
 * @brief Called when encoder turned right
 * @param  None
 * @retval None
 */
void button_right ()
{
	switch (screenMode)
	{
		case smRegular:
			lastNodeChangeTick = HAL_GetTick ();
			selectedNode++;
			if (selectedNode > MAX_NODES - 1)
				selectedNode = 0;
			break;

		case smSaveConfig:
			break;
		case smMainMenu:
			menuPosition = (menuPosition > 4) ? 0 : menuPosition + 1;
			break;
		case smRadioMenu:
			menuPosition = (menuPosition > 5) ? 0 : menuPosition + 1;
			break;
		case smBaseMenu:
			menuPosition = (menuPosition > 0) ? 0 : menuPosition + 1;
			break;
		case smNodeMenu:
			menuPosition = (menuPosition > 1) ? 0 : menuPosition + 1;
			break;
		case smNetworkMenu:
			menuPosition = (menuPosition > 4) ? 0 : menuPosition + 1;
			break;

		case smNodeAction:
			menuPosition = (menuPosition > 2) ? 0 : menuPosition + 1;
			break;

		case smFrequency:
			if (settings.realFrequency < MAX_FREQUENCY)
				settings.realFrequency += 50000;
			currentInterval = checkFreq (settings.realFrequency, settings.bw);

			if (currentInterval != BAD_INTERVAL && settings.power > legalFreq.interval[currentInterval].maxPower)
				settings.power = legalFreq.interval[currentInterval].maxPower;
			break;

		case smSf:
			if (settings.sf < 12)
				settings.sf++;
			break;

		case smBw:
			if (settings.bw < 9)
				settings.bw++;
			currentInterval = checkFreq (settings.realFrequency, settings.bw);
			break;

		case smSw:
			settings.syncWord++;
			if (settings.syncWord == 0x34)
				settings.syncWord = 0x35;
			break;

		case smCr:
			settings.cr = (settings.cr >= 4) ? 4 : settings.cr + 1;
			break;

		case smPreamble:
			settings.preamble = (settings.preamble >= MAX_PREAMBLE) ?
			MAX_PREAMBLE :
																																settings.preamble + 1;
			break;

		case smPower:
			settings.power = (settings.power >= 20) ? 20 : settings.power + 1;
			if (currentInterval != BAD_INTERVAL && settings.power > legalFreq.interval[currentInterval].maxPower)
				settings.power = legalFreq.interval[currentInterval].maxPower;
			break;

		case smInterval1:
			settings.warningDelay = (settings.warningDelay >= MAX_WARNING_DELAY) ?
			MAX_WARNING_DELAY :
																																							settings.warningDelay + 10000;
			break;

		case smInterval2:
			settings.superWarningDelay = (settings.superWarningDelay >= MAX_WARNING_DELAY) ?
			MAX_WARNING_DELAY :
																																												settings.superWarningDelay + 10000;
			break;

		case smWorkInterval:
			if (nodeSettings.workInterval < 600)
				nodeSettings.workInterval += 10;
			else
				nodeSettings.workInterval += 100;
			if (nodeSettings.workInterval > MAX_WORK_INTERVAL)
				nodeSettings.workInterval = MAX_WORK_INTERVAL;
			if (nodeSettings.workInterval > MAX_WORK_INTERVAL)
				nodeSettings.workInterval = MAX_WORK_INTERVAL;
			break;

		case smLed:
			nodeSettings.useLed = !nodeSettings.useLed;
			break;

		case smCleanNodeData:
			eraseNodeData ();
			break;

		case smRefreshNetworks:
			break;

		case smSelectNet:
			selectedNetwork = (selectedNetwork > 14) ? 0 : selectedNetwork + 1;
			break;

		case smSetWiFiPassword:
			settings.WiFiPass[cursorPos] = (settings.WiFiPass[cursorPos] >= 'z') ? ' ' : settings.WiFiPass[cursorPos] + 1;
			break;

		case smSetServerPassword:
			settings.ServerPass[cursorPos] = (settings.ServerPass[cursorPos] >= 'z') ? ' ' : settings.ServerPass[cursorPos] + 1;
			break;

		case smSetHost:
			settings.host[cursorPos] = (settings.host[cursorPos] >= 'z') ? ' ' : settings.host[cursorPos] + 1;
			break;

		case smSetBaseID:
			settings.baseID += pow10_ (6 - cursorPos);
			if (settings.baseID > MAX_BASEID)
				settings.baseID = MAX_BASEID;
			break;

		case smStatus:
		case smFirmware:
		case smMessageCounter:
			screenMode++;
			break;

		case smGraph:
			screenMode = smStatus;
			break;

		case smModeSelect:
			selectedMode++;
			if (selectedMode > testSignal)
				selectedMode = 0;
		default:
			break;
	}

	flag.lcdRefreshRequest = 1;
}

/**
 * @brief Called when encoder turned left
 * @param  None
 * @retval None
 */
void button_left ()
{
	switch (screenMode)
	{
		case smRegular:
			lastNodeChangeTick = HAL_GetTick ();
			selectedNode--;
			if (selectedNode < 0)
				selectedNode = MAX_NODES - 1;
			break;

		case smSaveConfig:
			break;
		case smMainMenu:
			menuPosition = (menuPosition < 1) ? 5 : menuPosition - 1;
			break;
		case smRadioMenu:
			menuPosition = (menuPosition < 1) ? 6 : menuPosition - 1;
			break;
		case smBaseMenu:
			menuPosition = (menuPosition < 1) ? 1 : menuPosition - 1;
			break;
		case smNodeMenu:
			menuPosition = (menuPosition < 1) ? 2 : menuPosition - 1;
			break;
		case smNetworkMenu:
			menuPosition = (menuPosition < 1) ? 5 : menuPosition - 1;
			break;

		case smNodeAction:
			menuPosition = (menuPosition < 1) ? 3 : menuPosition - 1;
			break;

		case smFrequency:
			if (settings.realFrequency > MIN_FREQUENCY)
				settings.realFrequency -= 50000;
			currentInterval = checkFreq (settings.realFrequency, settings.bw);

			if (currentInterval != BAD_INTERVAL && settings.power > legalFreq.interval[currentInterval].maxPower)
				settings.power = legalFreq.interval[currentInterval].maxPower;
			break;

		case smSf:
			if (settings.sf > 7)
				settings.sf--;
			break;

		case smBw:
			if (settings.bw > 1)
				settings.bw--;
			currentInterval = checkFreq (settings.realFrequency, settings.bw);
			break;

		case smSw:
			settings.syncWord--;
			if (settings.syncWord == 0x34)
				settings.syncWord = 0x33;
			break;

		case smCr:
			settings.cr = (settings.cr <= 1) ? 1 : settings.cr - 1;
			break;

		case smPreamble:
			settings.preamble = (settings.preamble <= MIN_PREAMBLE) ?
			MIN_PREAMBLE :
																																settings.preamble - 1;
			break;

		case smPower:
			settings.power = (settings.power <= 10) ? 10 : settings.power - 1;
			if (currentInterval != BAD_INTERVAL && settings.power > legalFreq.interval[currentInterval].maxPower)
				settings.power = legalFreq.interval[currentInterval].maxPower;
			break;

		case smInterval1:
			settings.warningDelay = (settings.warningDelay <= MIN_WARNING_DELAY) ?
			MIN_WARNING_DELAY :
																																							settings.warningDelay - 10000;
			break;

		case smInterval2:
			settings.superWarningDelay = (settings.superWarningDelay <= MIN_WARNING_DELAY) ?
			MIN_WARNING_DELAY :
																																												settings.superWarningDelay - 10000;
			break;

		case smWorkInterval:
			if (nodeSettings.workInterval >= 600)
				nodeSettings.workInterval -= 100;
			else
				nodeSettings.workInterval -= 10;
			if (nodeSettings.workInterval < MIN_WORK_INTERVAL)
				nodeSettings.workInterval = MIN_WORK_INTERVAL;
			break;

		case smLed:
			nodeSettings.useLed = !nodeSettings.useLed;
			break;

		case smCleanNodeData:
			break;

		case smRefreshNetworks:
			break;

		case smSelectNet:
			selectedNetwork = (selectedNetwork < 1) ? 15 : selectedNetwork - 1;
			break;

		case smSetWiFiPassword:
			settings.WiFiPass[cursorPos] = (settings.WiFiPass[cursorPos] <= ' ') ? 'z' : settings.WiFiPass[cursorPos] - 1;
			break;

		case smSetServerPassword:
			settings.ServerPass[cursorPos] = (settings.ServerPass[cursorPos] <= ' ') ? 'z' : settings.ServerPass[cursorPos] - 1;
			break;

		case smSetHost:
			settings.host[cursorPos] = (settings.host[cursorPos] <= ' ') ? 'z' : settings.host[cursorPos] - 1;
			break;

		case smSetBaseID:
			settings.baseID -= pow10_ (6 - cursorPos);
			if (settings.baseID <= 0)
				settings.baseID = 0;
			break;

		case smStatus:
			screenMode = smGraph;
			break;
		case smFirmware:
		case smMessageCounter:
		case smGraph:
			screenMode--;
			break;

		case smModeSelect:
			selectedMode--;
			if (selectedMode <= work)
				selectedMode = testSignal;
			break;
		default:
			break;
	}

	flag.lcdRefreshRequest = 1;
}

/**
 * @brief Asks node for his status
 * @param  None
 * @retval None
 */
void pingNode (int16_t selectedNode)
{
	txMes->adr = selectedNode;
	txMes->disarm = nodes[selectedNode].disarmRequest;
	txMes->message = MSG_DOWN_REQUEST;
	txMes->uplink = 0;
	SX127X_transmitAsync (&myRadio, 3);
}

/**
 * @brief Called when encoder pressed
 * @param  None
 * @retval None
 */
void button_ok ()
{
	if (screenMode == smModeSelect)
		{

			switch (selectedMode)
			{
				case work:
					screenMode = smRegular;
					break;
				case testColour:
					colorTest ();
					break;
				case testTransmitter:
					radioTestTransmit ();
					break;
				case testReceiver:
					radioTestReceive ();
					break;
				case testPing:
					radioTestPing ();
					break;
				case testSignal:
					radioSignalIndicator ();
					break;
				default:
					break;
			}

		}
	else if (screenMode == smRegular)
		{
			screenMode = smNodeAction;
		}
	else if (screenMode == smMainMenu)
		{
			switch (menuPosition)
			{
				case 0:
					screenMode = smRadioMenu;
					break;
				case 1:
					screenMode = smBaseMenu;
					break;
				case 2:
					screenMode = smNodeMenu;
					break;
				case 3:
					screenMode = smNetworkMenu;
					break;
				case 4:
					screenMode = smSaveConfig;
					break;
				case 5:
					screenMode = smStatus;
					break;
			}
			menuPosition = 0;
		}
	else if (screenMode == smRadioMenu)
		{

			switch (menuPosition)
			{
				case 0:
					screenMode = smFrequency;
					break;
				case 1:
					screenMode = smBw;
					break;
				case 2:
					screenMode = smSf;
					break;
				case 3:
					screenMode = smSw;
					break;
				case 4:
					screenMode = smCr;
					break;
				case 5:
					screenMode = smPreamble;
					break;
				case 6:
					screenMode = smPower;
					break;
			}
			menuPosition = 0;
		}
	else if (screenMode == smBaseMenu)
		{

			switch (menuPosition)
			{
				case 0:
					screenMode = smInterval1;
					break;
				case 1:
					screenMode = smInterval2;
					break;
			}
			menuPosition = 0;
		}
	else if (screenMode == smNodeMenu)
		{

			switch (menuPosition)
			{
				case 0:
					screenMode = smWorkInterval;
					break;
				case 1:
					screenMode = smLed;
					break;
				case 2:
					screenMode = smCleanNodeData;
					break;
			}
			menuPosition = 0;
		}
	else if (screenMode == smNetworkMenu)
		{
			switch (menuPosition)
			{
				case 0:
					screenMode = smRefreshNetworks;
					break;
				case 1:
					screenMode = smSelectNet;
					break;
				case 2:
					screenMode = smSetWiFiPassword;
					break;
				case 3:
					screenMode = smSetServerPassword;
					break;
				case 4:
					screenMode = smSetBaseID;
					break;
				case 5:
					screenMode = smSetHost;
					break;
			}
			menuPosition = 0;
		}
	else if (screenMode == smNodeAction)
		{
			switch (menuPosition)
			{
				case 0:
					screenMode = smRegular;
					pingNode (selectedNode);
					break;
				case 1:
					nodes[selectedNode].masked = (nodes[selectedNode].masked == 3) ? 0 : nodes[selectedNode].masked + 1;
					saveNodeData ();
					break;
				case 2:
					changeDisarmNode (selectedNode);
					saveNodeData ();
					break;
				case 3:
					configNodeViaUart (selectedNode);
					configTime = HAL_GetTick ();
					configStep = 1;
					break;
			}
		}
	else if (screenMode == smSaveConfig)
		{
			screenMode = smMainMenu;
			saveSettings ();
		}
	else if (screenMode == smRefreshNetworks)
		{
			NetRefreshNetworkList (&netHandler);
		}
	else if (screenMode == smSelectNet)
		{
			sprintf (settings.SSID, "%s", netHandler.nets[selectedNetwork]);
			screenMode = smNetworkMenu;
			cursorPos = 0;
		}
	else if (screenMode == smSetWiFiPassword)
		{
			if (cursorPos < 20 && settings.WiFiPass[cursorPos] != ' ' && settings.WiFiPass[cursorPos] != '\0')
				cursorPos++;
			else
				{
					settings.WiFiPass[cursorPos] = 0;
					screenMode = smNetworkMenu;
					cursorPos = 0;
				}
		}
	else if (screenMode == smSetHost)
		{
			if (cursorPos < 20 && settings.host[cursorPos] != ' ' && settings.host[cursorPos] != '\0')
				cursorPos++;
			else
				{
					settings.host[cursorPos] = 0;
					screenMode = smNetworkMenu;
					cursorPos = 0;
				}
		}
	else if (screenMode == smSetServerPassword)
		{
			if (cursorPos < 20 && settings.ServerPass[cursorPos] != ' ' && settings.ServerPass[cursorPos] != '\0')
				cursorPos++;
			else
				{
					settings.ServerPass[cursorPos] = 0;
					screenMode = smNetworkMenu;
					cursorPos = 0;
				}
		}
	else if (screenMode == smSetBaseID)
		{
			if (cursorPos < 7)
				cursorPos++;
			else
				screenMode = smNetworkMenu;
		}
	else if (screenMode == smMessageCounter)
		{
			screenMode = smMainMenu;
			menuPosition = 0;
		}
	else if (screenMode == smCleanNodeData)
		{
			eraseNodeData ();
		}
	else if (screenMode >= smFrequency && screenMode <= smPower)
		{
			screenMode = smRadioMenu;
		}

	flag.lcdRefreshRequest = 1;
}

/**
 * @brief Called when back button pressed
 * @param  None
 * @retval None
 */
void button_back ()
{
	ssd1306_Init ();
	UC1609_Init (&hspi2, CS_GPIO_Port, CS_Pin, CD_GPIO_Port, CD_Pin, 0, 0);
	menuPosition = 0;
	alarmOff ();
	switch (screenMode)
	{
		case smMainMenu:
		case smNodeAction:
			screenMode = smRegular;
			break;

		case smStatus:
		case smMessageCounter:
		case smFirmware:
		case smSaveConfig:
		case smRadioMenu:
		case smBaseMenu:
		case smNodeMenu:
		case smNetworkMenu:

			menuPosition = 0;
			screenMode = smMainMenu;
			break;

		case smFrequency:
		case smBw:
		case smSf:
		case smSw:
		case smCr:
		case smPreamble:
		case smPower:
			menuPosition = 0;
			screenMode = smRadioMenu;
			break;

		case smInterval1:
		case smInterval2:
			menuPosition = 0;
			screenMode = smBaseMenu;
			break;

		case smWorkInterval:
		case smLed:
		case smCleanNodeData:
			menuPosition = 0;
			screenMode = smNodeMenu;
			break;

		case smRefreshNetworks:
		case smSelectNet:
			menuPosition = 0;
			screenMode = smNetworkMenu;

		case smRegular:
		case smModeSelect:

			break;

		case smSetWiFiPassword:
		case smSetServerPassword:
		case smSetBaseID:
		case smSetHost:
			if (cursorPos)
				cursorPos--;
			else
				screenMode = smNetworkMenu;
			break;

		default:
			screenMode--;
	}

	flag.lcdRefreshRequest = 1;
}

void button0 ()
{
	pingNode (selectedNode);
}

void button1 ()
{
	lastNodeChangeTick = HAL_GetTick ();
	nodes[selectedNode].masked = (nodes[selectedNode].masked == 3) ? 0 : nodes[selectedNode].masked + 1;
	saveNodeData ();
	flag.lcdRefreshRequest = 1;
}

void button2 ()
{
	lastNodeChangeTick = HAL_GetTick ();
	changeDisarmNode (selectedNode);
	saveNodeData ();
	flag.lcdRefreshRequest = 1;

}

void button3 ()
{
lastNodeChangeTick=HAL_GetTick()-NODE_INFO_HOLD_TIME;
idleMode=!idleMode;
}

void drawGraph()
{
	ssd1306_Clean ();
#if MAX_NODES < 9
				for (int i = 0; i < MAX_NODES; i++)
					if (nodes[i].activated)
						sprintf (string[i], "%d", (int) ((nodes[i].NextMessageTick - HAL_GetTick ()) / 1000));
					else
						sprintf (string[i], "NA");

				ssd1306_DrawLine (26, 0, 26, 63);
				ssd1306_DrawDottedLine (106, 0, 106, 63);
				for (int i = 0; i < 8; i++)
					{
						ssd1306_GotoXY (0, i);
						ssd1306_PutString (string[i]);
					}
				for (int i = 0; i < MAX_NODES; i++)
					{
						if (((nodes[i].NextMessageTick - HAL_GetTick ()) < nodeSettings.workInterval * 2000
								|| HAL_GetTick () - nodes[i].NextMessageTick < nodeSettings.workInterval * 2000) && nodes[i].activated)
							{
								int8_t length = (nodes[i].NextMessageTick - HAL_GetTick ()) * 80 / 1000 / (int)nodeSettings.workInterval;
								if (length > 100)
									length = 100;
								if (length < -5)
									length = -5;
								ssd1306_DrawFilledRectangle (27, i * 8 + 3, 27 + length, i * 8 + 5);

							}
					}

				for (int i = 0; i < MAX_NODES; i++)
					if (nodes[i].activated)
						sprintf (string[i], "%d", (int) ((nodes[i].NextMessageTick - HAL_GetTick ()) / 1000));
					else
						sprintf (string[i], "NA");

				UC1609_DrawLine (26, 0, 26, 63);
				UC1609_DrawDottedLine (106, 0, 106, 63);
				for (int i = 0; i < 8; i++)
					{
						UC1609_GotoXY (0, i);
						UC1609_PutString (string[i]);
					}
				for (int i = 0; i < MAX_NODES; i++)
					{
						if (((nodes[i].NextMessageTick - HAL_GetTick ()) < nodeSettings.workInterval * 2000
								|| HAL_GetTick () - nodes[i].NextMessageTick < nodeSettings.workInterval * 2000) && nodes[i].activated)
							{
								int8_t length = (nodes[i].NextMessageTick - HAL_GetTick ()) * 80 / 1000 / (int)nodeSettings.workInterval;
								if (length > 100)
									length = 100;
								if (length < -5)
									length = -5;
								UC1609_DrawFilledRectangle (27, i * 8 + 3, 27 + length, i * 8 + 5);

							}
					}
#else
				ssd1306_Clean ();
				ssd1306_DrawLine (6, 0, 6, 63);
				ssd1306_DrawDottedLine (106, 0, 106, 63);
				for (int i = 0; i < MAX_NODES; i++)
					{
						int16_t delay = ((long) nodes[i].NextMessageTick - (long) HAL_GetTick ()) / 1000;

						if (nodes[i].activated && abs (delay) < nodeSettings.workInterval * 2)
							{
								int16_t length = delay * 100 / (int)nodeSettings.workInterval;
								if (length < -5)
									length = -5;
								if (length > 120)
									length = 120;
								ssd1306_DrawLine (6, i * 2, 6 + length, i * 2);
							}
					}



				UC1609_Clean();
				UC1609_DrawLine (26, 0, 26, 63);
				UC1609_DrawDottedLine (146, 0, 146, 63);
				for (int i = 0; i < MAX_NODES; i++)
					{
						int16_t delay = ((long) nodes[i].NextMessageTick - (long) HAL_GetTick ()) / 1000;

						if (nodes[i].activated && abs (delay) < nodeSettings.workInterval * 2)
							{
								int16_t length = delay * 120 / (int)nodeSettings.workInterval;
								if (length < -25)
									length = -25;
								if (length > 165)
									length = 165;
								UC1609_DrawLine (26, i * 2, 26 + length, i * 2);
							}
					}
#endif
				ssd1306_Update ();
				UC1609_UpdateScreen();
}
/**
 * @brief Forms and updates information on LCD, depends by selected screen mode
 * @param None
 * @retval None
 */
void lcdRoutine ()
{
	_Bool graphMode=false;
	clearStrings ();
	switch (screenMode)
	{
		case smRegular:

			if (HAL_GetTick () - lastNodeChangeTick < NODE_INFO_HOLD_TIME)
				{
					if (nodes[selectedNode].activated)
						{
							sprintf (string[0], "Node %d RSSI:%ddB", selectedNode, nodes[selectedNode].rssi);
							if (nodes[selectedNode].opened)
								sprintf (string[1], "Door: Opened");
							else
								sprintf (string[1], "Door: Closed");
							if (nodes[selectedNode].powered)
								sprintf (string[2], "Powered by PS");
							else
								sprintf (string[2], "Powered by battery");
							sprintf (string[3], "Voltage:%s V", floatToString2 (nodes[selectedNode].voltage));
							sprintf (string[4], "Temperature:%s C", floatToString1 (nodes[selectedNode].temperature));
							sprintf (string[5], "Next msg in %li", ((long) nodes[selectedNode].NextMessageTick - (long) HAL_GetTick ()) / 1000);
							sprintf (string[6], "Alarm: %s", maskNames[nodes[selectedNode].masked]);
						}
					else
						sprintf (string[0], "Node %d not activated", selectedNode);
				}
			else if(idleMode==0)
				{
					sprintf (string[0], "Up:%lu d & %lu:%02lu:%02lu ", upTime / 86400, upTime / 3600 % 24, upTime / 60 % 60, upTime % 60);
					sprintf (string[1], "Msg:%lu/%lu", receivedMesCntSuc, receivedMesCnt);
					sprintf (string[2], "Max delay: %lu", absoluteMaxDelay);
					if (lastMessageFrom != 255)
						{
							uint8_t next = nextNodeAwaited ();
							if (next != 255)
								sprintf (string[3], "Next %d in %li", next, ((long) nodes[next].NextMessageTick - (long) HAL_GetTick ()) / 1000);

							sprintf (string[4], "Last message from %d", lastMessageFrom);
							sprintf (string[5], "%s, %s", nodes[lastMessageFrom].opened ? "Door opened" : "Door closed",
												nodes[lastMessageFrom].powered ? "PS" : "Battery");
							sprintf (string[6], "T:%d, V:%s", (int) nodes[lastMessageFrom].temperature, floatToString1 (nodes[lastMessageFrom].voltage));
							sprintf (string[7], "Delay:%d", nodes[lastMessageFrom].delay);
						}

				}
			else if (idleMode==1)
				{
					graphMode=true;
					drawGraph();
				}

			break;
		case smMainMenu:
			sprintf (string[0], "Main Menu");
			sprintf (string[1], " Radio Settings");
			sprintf (string[2], " Base Settings");
			sprintf (string[3], " Node Settings");
			sprintf (string[4], " Network Settings");
			sprintf (string[5], " Save Settings");
			sprintf (string[6], " Info");
			string[1 + menuPosition][0] = 0x83;
			break;
		case smRadioMenu:
			sprintf (string[0], "Radio settings");
			sprintf (string[1], " Frequency");
			sprintf (string[2], " Bandwidth");
			sprintf (string[3], " Spreading Factor");
			sprintf (string[4], " SyncWord");
			sprintf (string[5], " Coding rate");
			sprintf (string[6], " Preamble");
			sprintf (string[7], " Power");
			string[1 + menuPosition][0] = 0x83;
			break;
		case smBaseMenu:
			sprintf (string[0], "Base settings");
			sprintf (string[1], " Warning Delay 1");
			sprintf (string[2], " Warning Delay 2");
			string[1 + menuPosition][0] = 0x83;
			break;
		case smNodeMenu:
			sprintf (string[0], "Node settings");
			sprintf (string[1], " Working Interval");
			sprintf (string[2], " Use LED");
			sprintf (string[3], " Clean saved data");
			string[1 + menuPosition][0] = 0x83;
			break;
		case smNetworkMenu:
			sprintf (string[0], "Network settings");
			sprintf (string[1], " Refresh Network List");
			sprintf (string[2], " Select Network");
			sprintf (string[3], " Set WiFi Password");
			sprintf (string[4], " Set Server Password");
			sprintf (string[5], " Set BaseID");
			sprintf (string[6], " Set Host");
			string[1 + menuPosition][0] = 0x83;
			break;

		case smFrequency:
			sprintf (string[0], "Frequency");
			sprintf (string[1], "%03ld.%02ld MHz", settings.realFrequency / 1000000, (settings.realFrequency / 10000) % 100);
			if (currentInterval != BAD_INTERVAL)
				{
					sprintf (string[2], "Power:%u dB", legalFreq.interval[currentInterval].maxPower);
					sprintf (string[3], "Air use.%u.%u%%", legalFreq.interval[currentInterval].maxAirUse / 10,
										legalFreq.interval[currentInterval].maxAirUse % 10);
				}
			else
				{
					sprintf (string[3], "Not within ISM Band!");
				}
			break;

		case smSf:
			sprintf (string[0], "Spreading");
			sprintf (string[1], "factor");
			sprintf (string[2], "%d", settings.sf);
			break;

		case smBw:
			sprintf (string[0], "Bandwidth");
			sprintf (string[1], "%s", bandwidthNames[settings.bw]);
			if (currentInterval != BAD_INTERVAL)
				{
					sprintf (string[2], "Power:%u dB", legalFreq.interval[currentInterval].maxPower);
					sprintf (string[3], "Air use.%u.%u%%", legalFreq.interval[currentInterval].maxAirUse / 10,
										legalFreq.interval[currentInterval].maxAirUse % 10);
				}
			else
				{
					sprintf (string[3], "Not within ISM Band!");
				}
			break;

		case smSw:
			sprintf (string[0], "SyncWord");
			sprintf (string[1], "0x%X", settings.syncWord);
			break;

		case smCr:
			sprintf (string[0], "Coding rate");
			sprintf (string[1], "4/%d", settings.cr + 4);
			break;

		case smPreamble:
			sprintf (string[0], "Preamble");
			sprintf (string[1], "%d sym", settings.preamble);
			break;

		case smPower:
			sprintf (string[0], "Transmission power");

			sprintf (string[1], "%d dBm (%d mW)", settings.power, mW[settings.power - 10]);
			if (currentInterval != BAD_INTERVAL)
				{
					sprintf (string[2], "For this frequency");
					sprintf (string[3], "Max:%d dBm", legalFreq.interval[currentInterval].maxPower);
				}
			else
				{
					sprintf (string[3], "Not within ISM band");
				}
			break;

		case smInterval1:
			sprintf (string[0], "Level 1");
			sprintf (string[1], "Silence Alarm");
			sprintf (string[2], "%lu s", settings.warningDelay / 1000);
			break;

		case smInterval2:
			sprintf (string[0], "Level 2");
			sprintf (string[1], "Silence alarm");
			sprintf (string[2], "%lu s", settings.superWarningDelay / 1000);
			break;

		case smWorkInterval:
			sprintf (string[0], "Send status every");
			sprintf (string[1], "%lu s", nodeSettings.workInterval);
			break;

		case smLed:
			sprintf (string[0], "Node LED use");
			if (nodeSettings.useLed)
				sprintf (string[1], "On");
			else
				sprintf (string[1], "Off");
			break;
		case smCleanNodeData:
			sprintf (string[0], "Clean saved");
			sprintf (string[1], "Node data");
			break;
		case smRefreshNetworks:
			sprintf (string[0], "Refresh Network");
			sprintf (string[1], "List");
			if (netHandler.netRefreshInProgress)
				sprintf (string[2], "In progress...");
			if (netHandler.netRefreshDone)
				sprintf (string[3], "Found %d Nets", netHandler.netCount);
			break;

		case smSelectNet:
			sprintf (string[0], "Select WLAN");
			if (netHandler.nets[selectedNetwork][0] == 0)
				sprintf (string[1], "Slot %d is empty", selectedNetwork);
			else
				sprintf (string[1], "%s", netHandler.nets[selectedNetwork]);
			break;

		case smSetWiFiPassword:
			sprintf (string[0], "Set WiFi password");
			sprintf (string[1], settings.WiFiPass);
			setCursor (string[2], cursorPos);
			if (settings.WiFiPass[cursorPos] == ' ' || settings.WiFiPass[cursorPos] == '\0')
				sprintf (string[3], "Done");
			break;

		case smSetServerPassword:
			sprintf (string[0], "Set Server password");
			sprintf (string[1], settings.ServerPass);
			setCursor (string[2], cursorPos);
			if (settings.ServerPass[cursorPos] == ' ' || settings.ServerPass[cursorPos] == '\0')
				sprintf (string[3], "Done");
			break;

		case smSetHost:
			sprintf (string[0], "Set Host");
			sprintf (string[1], settings.host);
			setCursor (string[2], cursorPos);
			if (settings.host[cursorPos] == ' ' || settings.host[cursorPos] == '\0')
				sprintf (string[3], "Done");
			break;

		case smSetBaseID:
			sprintf (string[0], "Set Base ID");
			sprintf (string[1], "%07lu", settings.baseID);
			setCursor (string[2], cursorPos);
			break;

		case smSaveConfig:
			sprintf (string[0], "Refresh setings?");
			sprintf (string[1], "OK:Yes BACK:No");
			break;

		case smMessageCounter:
			sprintf (string[0], "Total received:");
			sprintf (string[1], "%lu/%lu", receivedMesCntSuc, receivedMesCnt);
			break;
		case smFirmware:
			sprintf (string[0], "Firmware: ");
			sprintf (string[1], "release %lx", version);
			break;
		case smStatus:
			{

				sprintf (string[0], "Uptime:");
				sprintf (string[1], "%lu d & %lu:%02lu:%02lu ", upTime / 86400, upTime / 3600 % 24, upTime / 60 % 60, upTime % 60);
				sprintf (string[2], "RX use: %s", floatToString2 (airUseForRx));
				sprintf (string[3], "TX use: %s", floatToString2 (airUseForTx));
				sprintf (string[4], "WiFI:%d,Connected:%d", netHandler.plugged, netHandler.connected);
				sprintf (string[5], "Local IP:%s", netHandler.ip);
				sprintf (string[6], "Online:%d", netHandler.online);
				sprintf (string[7], "TX:%d", netHandler.TX);

				break;
			}
		case smGraph:
			{
				graphMode=true;
drawGraph();
			}

			break;
		case smNodeAction:
			sprintf (string[0], "   Node Actions:");
			sprintf (string[1], " Ping");
			sprintf (string[2], " Alarm: %s", maskNames[nodes[selectedNode].masked]);
			sprintf (string[3], nodes[selectedNode].disarmRequest ? " Disarmed" : " Armed");
			sprintf (string[4], " Config Node");
			string[1 + menuPosition][0] = 0x83;
			break;

		case smModeSelect:

			clearStrings ();

			sprintf (string[0], " Work");
			sprintf (string[1], " Color test");
			sprintf (string[2], " Transmitter test");
			sprintf (string[3], " Receiver Test");
			sprintf (string[4], " Ping test");
			sprintf (string[5], " RSSI Indicator");
			string[selectedMode][0] = 0x83;

			break;
	}
	if (!graphMode)
		updateLcd ();
}

/**
 * @brief Test mode privider
 * Need for WS2812 RGB led color test
 * @param  None
 * @retval None
 */
void colorTest ()
{
	flag.lcdRefreshRequest = 1;
	while (1)
		{
			static uint8_t red = 0;
			static uint8_t green = 0;
			static uint8_t blue = 0;
			static uint8_t currentCol = 0;

			if (flag.encInc)
				{
					flag.encInc = 0;
					switch (currentCol)
					{
						case 0:
							red++;
							break;
						case 2:
							blue++;
							break;
						case 1:
							green++;
							break;
					}
					flag.lcdRefreshRequest = 1;
				}

			if (flag.encDec)
				{
					flag.encDec = 0;
					switch (currentCol)
					{
						case 0:
							red--;
							break;
						case 2:
							blue--;
							break;
						case 1:
							green--;
							break;
					}
					flag.lcdRefreshRequest = 1;
				}

			if (flag.encOk)
				{
					flag.encOk = 0;
					currentCol = (++currentCol > 2) ? 0 : currentCol;
					flag.lcdRefreshRequest = 1;
				}

			if (flag.lcdRefreshRequest == 1)
				{
					flag.lcdRefreshRequest = 0;
					clearStrings ();
					sprintf (string[0], "Color test");
					sprintf (string[1], " Red:  %3d %02X", red, red);
					sprintf (string[2], " Green:%3d %02X", green, green);
					sprintf (string[3], " Blue: %3d %02X", blue, blue);
					string[currentCol + 1][0] = 0x83;

					updateLcd ();
				}
			wsSetColor (0, red, green, blue);
			wsPrepareArray ();
		}
}

/**
 * @brief Test mode provider
 * continuously transmits messages with current radio settings
 * Transmittion power can be cyclic increased or static 20dbm
 * @param  None
 * @retval None
 */
void radioTestTransmit ()
{
	static uint8_t radioTxTestMode = 0; //0-ladder //1 - 10 // 2 - 11...
	static uint8_t power = 10;

	SX127X_defaultConfig (&myRadio);
	tryLoadSettings ();
	SX127X_config (&myRadio);
	flag.lcdRefreshRequest = 1;
	while (1)
		{
			uint8_t i;
			for (i = 0; i < 9; i++)
				{
					if (power - 12 == i)
						wsSetColor (i, GREEN);
					else
						wsSetColor (i, BLACK);
				}
			wsPrepareArray ();

			if (myRadio.status != TX)
				{
					if (radioTxTestMode == 0)
						power = (power + 1 > 20) ? 10 : power + 1;
					else
						power = radioTxTestMode + 9;
					myRadio.power = power;
					SX127X_config (&myRadio);
					myRadio.txBuf[0] = 'a' + power - 10;
					SX127X_transmitAsync (&myRadio, 3);
					flag.lcdRefreshRequest = 1;
				}

			if (flag.encInc || flag.encDec)
				{
					if (flag.encInc)
						radioTxTestMode = (radioTxTestMode == 11) ? 0 : radioTxTestMode + 1;
					if (flag.encDec)
						radioTxTestMode = (radioTxTestMode == 0) ? 11 : radioTxTestMode - 1;
					flag.encInc = 0;
					flag.encDec = 0;
					flag.lcdRefreshRequest = 1;
				}

			SX127X_Routine (&myRadio);
			if (flag.lcdRefreshRequest)
				{
					flag.lcdRefreshRequest = 0;

					clearStrings ();
					sprintf (string[0], "Transmitter");
					sprintf (string[1], "Mode:");
					if (!radioTxTestMode)
						sprintf (string[3], "Cyclic");
					else
						sprintf (string[3], "Constant %u mw", radioTxTestMode + 9);
					sprintf (string[2], "Pow:%d dBm", power);
					updateLcd ();
				}
		}
}

/**
 * @brief Test mode provider
 * continuously receives messages with current radio settings
 * Transmission powerd displayed by LCD and led
 * @param  None
 * @retval None
 */
void radioTestReceive ()
{
	SX127X_defaultConfig (&myRadio);
	tryLoadSettings ();
	SX127X_config (&myRadio);
	myRadio.alwaysRX = 1;

	flag.lcdRefreshRequest = 1;
	while (1)
		{
			static uint16_t ledPower[9];
			uint8_t i;
			uint32_t lastMessageReceived = 0;
			char message[20] = { 0, };

			SX127X_Routine (&myRadio);

			if (flag.uartGotMessage)
				{
					flag.uartGotMessage = 0;

					uartReceiveHandler ();
				}

//LCD Handler

			if (myRadio.readBytes)
				{
					if (myRadio.badCrc)
						{
							myRadio.readBytes = 0;
							continue;
						}
					memcpy (message, myRadio.rxBuf, myRadio.readBytes);
					ledPower[(int) (myRadio.rxBuf[0] - 'a')] = 3000;
					//power=myRadio.rxBuf[0]+10-'a'/2;
					myRadio.readBytes = 0;
					lastMessageReceived = HAL_GetTick ();
					flag.lcdRefreshRequest = 1;
				}

//Led handler
			if (myRadio.signalDetected)
				wsSetColor (0, 0, 0, 20);
			else
				wsSetColor (0, 0, 0, 0);
			if (HAL_GetTick () - lastMessageReceived < 1000)
				wsSetColor (1, 0, 20, 0);
			else
				wsSetColor (1, 0, 0, 0);

			for (i = 2; i < 9; i++)
				{
					wsSetColor (i, 0, ledPower[i] / 256, 0);
				}
			wsPrepareArray ();
			for (i = 0; i < 9; i++)
				ledPower[i] = ledPower[i] ? ledPower[i] - 1 : 0;

			if (flag.lcdRefreshRequest)
				{
					flag.lcdRefreshRequest = 0;

					clearStrings ();
					sprintf (string[0], "Receiver RSSI:%d dB", SX127X_RSSI_Pack (&myRadio));
					memcpy (string[1], message, 14);
					sprintf (string[2], "%02x %02x %02x %02x %02x", message[0], message[1], message[2], message[3], message[4]);
					sprintf (string[3], "%02x %02x %02x %02x %02x", message[5], message[6], message[7], message[8], message[9]);
					updateLcd ();
				}
		}
}

/**
 * @brief Test mode provider
 * Pings Base in manual or automatic mode
 * @param  None
 * @retval None
 */
void radioTestPing ()
{
	uint8_t autoPing = 0;
	uint32_t lastSend = 0;
	uint32_t lastReceived = 0;
	uint32_t lastLcdUpdate = 0;
	uint8_t i;
	uint32_t sendCount = 0;
	uint32_t receivedCount = 0;
	SX127X_defaultConfig (&myRadio);
	tryLoadSettings ();
	SX127X_config (&myRadio);
	myRadio.alwaysRX = 1;

	txMes->adr = 0;
	txMes->disarm = 0;
	txMes->uplink = 0;
	txMes->message = MSG_DOWN_REQUEST;

	while (1)
		{
			HAL_UART_Receive_IT (&huart1, &uartIn, 1);

			if (flag.encInc || flag.encDec)
				{
					flag.encInc = 0;
					flag.encDec = 0;
					autoPing = ~autoPing;
					flag.lcdRefreshRequest = 1;
				}

			if (myRadio.readBytes > 0)
				{
					if (myRadio.badCrc != 0)
						{
							myRadio.readBytes = 0;
							continue;
						}
					receivedCount++;
					lastReceived = HAL_GetTick ();
					myRadio.readBytes = 0;
				}

			if (HAL_GetTick () - lastSend > 2000 && (autoPing || flag.encOk))
				{
					sendCount++;
					flag.encOk = 0;
					SX127X_transmitAsync (&myRadio, 3);
					lastSend = HAL_GetTick ();
				}

			SX127X_Routine (&myRadio);

//Handle LED
			for (i = 0; i < 9; i++)
				wsSetColor (i, BLACK);
			if (myRadio.status == TX)
				{
					wsSetColor (1, MAGENTA);
					wsSetColor (2, MAGENTA);
					wsSetColor (3, MAGENTA);
					wsSetColor (4, MAGENTA);
				}
			if (HAL_GetTick () - lastReceived < 600 && lastReceived)
				{
					wsSetColor (5, GREEN);
					wsSetColor (6, GREEN);
					wsSetColor (7, GREEN);
					wsSetColor (8, GREEN);
				}
			if (HAL_GetTick () - lastUartConnect < 200)
				{
					wsSetColor (0, YELLOW);
				}
			wsPrepareArray ();

//Handle UART
			if (HAL_GetTick () - lastLcdUpdate > 300)
				flag.lcdRefreshRequest = 1;

			if (flag.uartGotMessage)
				{
					flag.uartGotMessage = 0;
					lastUartConnect = HAL_GetTick ();

					uartReceiveHandler ();
				}

//Handle LCD
			if (flag.lcdRefreshRequest)
				{
					flag.lcdRefreshRequest = 0;

					float sucess;
					if (sendCount != 0)
						sucess = (float) receivedCount / (float) sendCount;
					else
						sucess = 0;

					lastLcdUpdate = HAL_GetTick ();
					clearStrings ();

					if (autoPing)
						sprintf (string[0], "Ping auto");
					else
						sprintf (string[0], "Ping manual");
					sprintf (string[1], "send %lu", sendCount);
					sprintf (string[2], "received %lu", receivedCount);
					sprintf (string[3], "%d%% sucess", (int) (sucess * 100));
					updateLcd ();
				}
		}
}

/**
 * @brief Test mode provider
 * Shows current RSSI
 * @param  None
 * @retval None
 */
void radioSignalIndicator ()
{
	uint8_t i;
	int16_t rssi;
	int16_t scale;
	SX127X_defaultConfig (&myRadio);
	tryLoadSettings ();
	SX127X_config (&myRadio);
	while (1)
		{
			rssi = SX127X_RSSI (&myRadio);
			scale = (rssi > -120) ? 120 + rssi : 0;
			clearStrings ();
			sprintf (string[0], "RSSI Meter");
			sprintf (string[1], "RSSI:%4d C:%d", rssi, scale);
			SX127X_Routine (&myRadio);
			updateLcd ();
			wsSetColor (0, BLACK);
			if (myRadio.signalDetected)
				wsSetColor (0, CYAN);
			for (i = 0; i < 8; i++)
				{
					if (scale / 10 > i)
						wsSetColor (8 - i, 0, 10, 0);
					else if (scale / 10 < i)
						wsSetColor (8 - i, 0, 0, 0);
					else
						wsSetColor (8 - i, 0, scale % 10, 0);

				}
			wsPrepareArray ();
		}
}
/**
 * @brief Sets all node settings to default values
 * @param  None
 * @retval None
 */
void defaultNodeSettings ()
{
	nodeSettings.bw = SX127X_LORA_BW_125KHZ;
	nodeSettings.cr = SX127X_CR_4_8;
	nodeSettings.power = SX127X_POWER_20DBM;
	nodeSettings.preamble = 5;
	nodeSettings.realFrequency = DEF_FREQUENCY;
	nodeSettings.sf = SX127X_LORA_SF_12;
	nodeSettings.sw = 0x1;
	nodeSettings.useLed = true;
	nodeSettings.voltageTreshold = 2.5f;
	nodeSettings.workInterval = 60;
}

/**
 * @brief Initiates array of available frequencies
 * @param  None
 * @retval None
 */
void initLegalFreq ()
{
	legalFreq.intervalCount = 3;

	legalFreq.interval[0].minFreq = 864000000;
	legalFreq.interval[0].maxFreq = 865000000;
	legalFreq.interval[0].maxPower = 14; //25mW
	legalFreq.interval[0].maxAirUse = 1; //0.1%

	legalFreq.interval[1].minFreq = 866000000;
	legalFreq.interval[1].maxFreq = 868000000;
	legalFreq.interval[1].maxPower = 14;
	legalFreq.interval[1].maxAirUse = 10; //1%

	legalFreq.interval[2].minFreq = 868700000;
	legalFreq.interval[2].maxFreq = 869200000;
	legalFreq.interval[2].maxPower = 20;
	legalFreq.interval[2].maxAirUse = 100; //10%
}

/**
 * @brief returns the number of the interval
 * that the frequency is included in
 * @param  real frequency in Hz, bandwidth index
 * @retval returns interval
 */
uint8_t checkFreq (uint32_t frequency, uint8_t bw)
{
	uint8_t i;
	for (i = 0; i < legalFreq.intervalCount; i++)
		{
			if (frequency - bandWidth[bw] / 2 >= legalFreq.interval[i].minFreq && frequency - bandWidth[bw] / 2 <= legalFreq.interval[i].maxFreq
					&& frequency + bandWidth[bw] / 2 >= legalFreq.interval[i].minFreq && frequency + bandWidth[bw] / 2 <= legalFreq.interval[i].maxFreq)
				{
					return i;
				}
		}
	return BAD_INTERVAL;
}

void backButtonHoldRoutine ()
{
	if (HAL_GPIO_ReadPin (B2_GPIO_Port, B2_Pin) == false && screenMode == smRegular)
		menuDelayCounter++;
	else
		menuDelayCounter = 0;

	if (menuDelayCounter > 2000)
		{
			menuDelayCounter = 0;
			screenMode = smMainMenu;
			flag.lcdRefreshRequest = 1;
		}

}

void halfSecondRoutine ()
{
	static uint32_t lastHalf = 0;
	if (lastHalf != HAL_GetTick () / 500 && ((screenMode >= smStatus && screenMode <= smMessageCounter)))
		{
			lastHalf = HAL_GetTick () / 500;

			flag.lcdRefreshRequest = 1;
		}
}

void secondRoutine ()
{
	static uint32_t lastSecond = 0;
	if (HAL_GetTick () / 1000 != lastSecond)
		{
			lastSecond = HAL_GetTick () / 1000;
			if (screenMode == smRegular || screenMode == smGraph)
				flag.lcdRefreshRequest = 1;
		}
}

uint8_t checkNodeConfigure ()
{
	return (settings.bw == nodeSettings.bw && settings.cr == nodeSettings.cr && settings.sf == nodeSettings.sf
			&& settings.realFrequency == nodeSettings.realFrequency && settings.preamble == nodeSettings.preamble && settings.syncWord == nodeSettings.sw
			&& selectedNode == nodeSettings.nodeNum);
}

void nodeConfigureRoutine ()
{
	if (HAL_GetTick () - configTime > 500 && configStep == 1)
		{
			clearStrings ();
			if (checkNodeConfigure ())
				{
					sprintf (string[0], "Sucess");
				}
			else
				{
					sprintf (string[0], "Failure");
				}
			updateLcd ();
			configStep = 2;
		}

	if (HAL_GetTick () - configTime > 1500 && configStep == 2)
		{
			configStep = 0;
			flag.lcdRefreshRequest = 1;
		}
}

void RadioInit ()
{
	SX127X_dio_t nss;
	SX127X_dio_t reset;
	SX127X_defaultConfig (&myRadio);

	nss.pin = NSS_Pin;
	nss.port = NSS_GPIO_Port;
	reset.pin = RESET_Pin;
	reset.port = RESET_GPIO_Port;
	SX127X_PortConfig (&myRadio, reset, nss, &hspi1);
	SX127X_init (&myRadio);
}

void airCounter ()
{
	static uint32_t upTimeMs;
	static uint32_t transmittingTimeMs;
	static uint32_t receivingTimeMs;
	static uint32_t lastTick;

	uint32_t delta = HAL_GetTick () - lastTick;
	upTimeMs += delta;
	if (myRadio.signalDetected)
		receivingTimeMs += delta;
	if (myRadio.status == TX)
		transmittingTimeMs += delta;
	upTime += upTimeMs / 1000;
	upTimeMs %= 1000;
	if (receivingTimeMs > 1000)
		{
			receivingTime += receivingTimeMs / 1000;
			receivingTimeMs %= 1000;
		}
	if (transmittingTimeMs > 1000)
		{
			transmittingTime += transmittingTimeMs / 1000;
			transmittingTimeMs %= 1000;
		}
	airUseForRx = (receivingTime == 0) ? 0 : (float) receivingTime / (float) upTime;
	airUseForTx = (transmittingTime == 0) ? 0 : (float) transmittingTime / (float) upTime;
	lastTick = HAL_GetTick ();

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

//Pass default config to avoid WD initiate
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

//General Init
	DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP_Msk | DBGMCU_CR_DBG_WWDG_STOP_Msk;
	HAL_Delay (300);
	ssd1306_Init ();

	UC1609_Init (&hspi2, CS_GPIO_Port, CS_Pin, CD_GPIO_Port, CD_Pin, 0, 0);

	wsInit (&htim2, TIM_CHANNEL_2);
	uartInit (&huart1);
	HAL_ADCEx_Calibration_Start (&hadc2);
	HAL_Delay (100);
	HAL_ADC_Start (&hadc2);

	//Using default settings
	defaultSettings ();
	RadioInit ();
	initLegalFreq ();
	memset (nodes, 0, sizeof(nodes));
	loadNodeData ();
	defaultNodeSettings ();

	ShowLogo ();

	clearStrings ();

	if (tryLoadSettings ())
		{
			sprintf (string[0], "Settings loaded");
			sprintf (string[1], "from flash");
		}
	else
		{
			sprintf (string[0], "Failed to load");
			sprintf (string[1], "settings from flash");
			sprintf (string[2], "Using defaults");
		}
	updateLcd ();
	HAL_Delay (1000);
	flag.lcdRefreshRequest = 1;

	myRadio.alwaysRX = true;   //Always listen for AIR

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	if (HAL_GPIO_ReadPin (B2_GPIO_Port, B2_Pin) == GPIO_PIN_RESET)
		screenMode = smModeSelect;

	while (1)
		{

			HAL_IWDG_Refresh (&hiwdg);

			blinkProvider = HAL_GetTick () % 1000 > 500;
			fastBlinkProvider = HAL_GetTick () % 70 > 35;

			analogRoutine ();
			backButtonHoldRoutine ();
			halfSecondRoutine ();
			secondRoutine ();
			alarmRoutine ();
			ledRoutine ();
			airCounter ();
			NetRoutine (&netHandler);
			SX127X_Routine (&myRadio);

			if (flag.lcdRefreshRequest)
				{
					flag.lcdRefreshRequest = 0;
					lcdRoutine ();
				}

			if (myRadio.readBytes)
				{
					handleMessage (myRadio.readBytes);
					myRadio.readBytes = 0;
				}

			if (flag.encDec)
				{
					flag.encDec = 0;
					button_left ();
				}

			if (flag.encInc)
				{
					flag.encInc = 0;
					button_right ();
				}

			if (flag.encOk)
				{
					flag.encOk = 0;
					button_ok ();
				}

			if (flag.back == 1)
				{
					flag.back = 0;
					button_back ();
				}

			if (flag.analog0 == 1)
				{
					flag.analog0 = 0;
					button0 ();
				}

			if (flag.analog1 == 1)
				{
					flag.analog1 = 0;
					button1 ();
				}
			if (flag.analog2 == 1)
				{
					flag.analog2 = 0;

					button2 ();
				}
			if (flag.analog3 == 1)
				{
					flag.analog3 = 0;
					button3 ();
				}

			if (screenMode == smModeSelect)
				continue;

			if (flag.uartGotMessage)
				{
					flag.uartGotMessage = 0;

					uartReceiveHandler ();
				}

			if (flag.saveSettings)
				{
					flag.saveSettings = 0;

					saveSettings ();
				}

			if (flag.sendConfig)
				{
					flag.sendConfig = 0;
					sendConfig ();
				}

			if (configStep)
				{
					nodeConfigureRoutine ();
				}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 89;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_Pin|NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|CD_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RESET_Pin NSS_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin B2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B3_Pin B4_Pin */
  GPIO_InitStruct.Pin = B3_Pin|B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin CD_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = CS_Pin|CD_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

	while (1)
		;
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
