#include "main.h"

extern flag_t flag;
extern UART_HandleTypeDef huart1;

extern char error[40];

extern baseSettings_t settings;
extern nodeSettings_t nodeSettings;

extern nodeHandler_t nodes[MAX_NODES];
extern uint32_t lastUartConnect;
extern uint32_t configTime;
extern uint8_t configStep;
extern uint8_t uartIn;
extern uint16_t version;
extern NetHandler_t netHandler;

uint8_t uartRx[128];
uint8_t uartPos;
uint8_t len;
uint8_t nodeNum;

void uartInit() {
	HAL_UART_Receive_IT(&huart1, &uartIn, 1);
}

void readByte(void) {
	switch (uartIn) {
	case '<':
		uartPos = 0;
		break;

	case '>':
		len = uartPos;
		uartRx[uartPos] = 0;
		flag.uartGotMessage = 1;
		uartPos = 0;
		break;

	default:
		uartRx[uartPos++] = uartIn;
		break;
	}
	if (uartPos > 126)
		uartPos = 126;
}

void uartReceiveHandler() {
	char tempString[20];

	uint8_t l = len - 1;
	uint8_t *ptr = uartRx + 1;

	if (HAL_GetTick() - configTime > 500) //First half second after node configuring saving incoming data into node var
		switch (uartRx[0]) {
		case UART_FREQUENCY:
			settings.realFrequency = DecToInt(ptr, l);
			break;

		case UART_SF:
			settings.sf = DecToInt(ptr, l);
			break;

		case UART_BW:
			settings.bw = DecToInt(ptr, l);
			break;

		case UART_SYNCWORD:
			settings.syncWord = HexToInt(ptr, l);
			break;

		case UART_PREAMBLE:
			settings.preamble = DecToInt(ptr, l);
			break;

		case UART_CR:
			settings.cr = DecToInt(ptr, l);
			break;

		case UART_POWER:
			settings.power = DecToInt(ptr, l);
			break;

		case UART_SAVE:
			flag.saveSettings = 1;
			break;

		case UART_READ:
			flag.sendConfig = 1;
			break;

		case UART_CALL:
			sprintf(tempString, "<aBv%d>", version);
			HAL_UART_Transmit(&huart1, (uint8_t*) tempString,
					strlen(tempString), 100);
			break;

		case UART_WARNING_DELAY:
			settings.warningDelay = 1000 * DecToInt(ptr, l);
			break;

		case UART_SUPER_WARNING_DELAY:
			settings.superWarningDelay = 1000 * DecToInt(ptr, l);
			break;

		case UARTW_SSID:
			memset(settings.SSID, 0, sizeof(settings.SSID));
			memcpy(settings.SSID, ptr, l);
			break;

		case UARTW_WIFIPASS:
			memcpy(settings.WiFiPass, ptr, l);
			break;

		case UARTW_WIFION:
			netHandler.lastStatusCheck = HAL_GetTick();
			netHandler.plugged = true;
			flag.lcdRefreshRequest = true;
			void NetTurnOffDebug(NetHandler_t *h);
			break;

		case UARTW_CONNECTED:
			netHandler.lastStatusCheck = HAL_GetTick();
			netHandler.connected = ptr[0];
			flag.lcdRefreshRequest = true;
			break;

		case UARTW_ONLINE:
			netHandler.lastStatusCheck = HAL_GetTick();
			netHandler.online = ptr[0];
			flag.lcdRefreshRequest = true;
			break;

		case UARTW_WIFICOUNT:
			netHandler.lastStatusCheck = HAL_GetTick();
			netHandler.netCount = DecToInt(ptr, l);
			netHandler.netRefreshDone = true;
			netHandler.netRefreshInProgress = false;
			flag.lcdRefreshRequest = 1;
			break;
		case UARTW_WIFISSID:
			netHandler.lastStatusCheck = HAL_GetTick();
			{
				int i = 0;
				while (i < 16) {
					if (netHandler.nets[i][0] == 0) {
						sprintf(netHandler.nets[i], "%s", ptr);
						break;
					} else
						i++;
				}
			}
			break;

		case UARTW_HTTPRESPONSE:
		case UARTW_RESPONSE:
		case UARTW_RESPONSE1:
		case UARTW_HTTPFAIL:
			netHandler.TX = false;
			netHandler.lastStatusCheck = HAL_GetTick();
			sprintf(netHandler.response, "%s", ptr);
			netHandler.TX = false;
			flag.lcdRefreshRequest = true;
			break;

		case UARTW_IP:
			netHandler.lastStatusCheck = HAL_GetTick();
			sprintf(netHandler.ip, "%s", ptr);
			netHandler.connected = true;
			flag.lcdRefreshRequest = true;
			break;

		default:
			HAL_UART_Transmit(&huart1, (uint8_t*) "//Bad Format", 10, 100);
			break;
		}
	else {
		switch (uartRx[0]) {
		case UART_FREQUENCY:
			nodeSettings.realFrequency = DecToInt(ptr, l);
			break;

		case UART_SF:
			nodeSettings.sf = DecToInt(ptr, l);
			break;

		case UART_BW:
			nodeSettings.bw = DecToInt(ptr, l);
			break;

		case UART_SYNCWORD:
			nodeSettings.sw = HexToInt(ptr, l);
			break;

		case UART_PREAMBLE:
			nodeSettings.preamble = DecToInt(ptr, l);
			break;

		case UART_CR:
			nodeSettings.cr = DecToInt(ptr, l);
			break;

		case UART_POWER:
			nodeSettings.power = DecToInt(ptr, l);
			break;

		case UART_NODENUM:
			nodeNum = DecToInt(ptr, l);
			break;

		case UART_USELED:
			nodeSettings.useLed = DecToInt(ptr, l);
			break;

		case UART_WORKING_INTERVAL:
			nodeSettings.workInterval = DecToInt(ptr, l);
			break;

		default:
			HAL_UART_Transmit(&huart1, (uint8_t*) "Bad Format", 10, 100);
			break;
		}
	}
}

void sendConfig(void) {
	lastUartConnect = HAL_GetTick();
	printf("<1%ld>\n", settings.realFrequency);
	printf("<2%u>\n", settings.sf);
	printf("<3%u>\n", settings.bw);
	printf("<4%X>\n", settings.syncWord);
	printf("<5%u>\n", settings.power);
	printf("<8%u>\n", settings.preamble);
	printf("<9%u>\n", settings.cr);
	printf("<q%lu>\n", settings.warningDelay / 1000);
	printf("<w%lu>\n", settings.superWarningDelay / 1000);
	printf("<Z%s>\n", settings.SSID);
	printf("<X%s>\n", settings.WiFiPass);
}

void configNodeViaUart(uint8_t nodeNum) {
	memset(&nodeSettings, 0, sizeof(nodeSettings_t));
	printf("<1%ld\n>", settings.realFrequency);
	HAL_Delay(5);
	printf("<2%u>\n", settings.sf);
	HAL_Delay(5);
	printf("<3%u>\n", settings.bw);
	HAL_Delay(5);
	printf("<4%X>\n", settings.syncWord);
	HAL_Delay(5);
	printf("<5%u>\n", settings.power);
	HAL_Delay(5);
	printf("<8%u>\n", settings.preamble);
	HAL_Delay(5);
	printf("<9%u>\n", settings.cr);
	HAL_Delay(5);
	printf("<n%u>\n", nodeNum);
	HAL_Delay(5);
	printf("<i%lu>\n", nodeSettings.workInterval);
	HAL_Delay(5);
	printf("<L%u>\n", nodeSettings.useLed);
	HAL_Delay(5);
	printf("<S>\n");
	HAL_Delay(10);
	printf("<R>\n");
	configTime = HAL_GetTick();
	lastUartConnect = HAL_GetTick();
}

