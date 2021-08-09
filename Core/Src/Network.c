#include "main.h"

extern baseSettings_t settings;
extern UART_HandleTypeDef huart1;
extern NetHandler_t netHandler;

uint32_t lastStatusCheck = 0;

void SendCommand (char com, char *data)
{
	char tmpStr[256] = { 0, };
	uint8_t len;
	sprintf (tmpStr, "<%c%s", com, data);
	len = strlen (tmpStr);
	tmpStr[len] = '>';
	HAL_UART_Transmit (&huart1, (uint8_t*) tmpStr, len + 1, 1000);
}

void NetRoutine (NetHandler_t *h)
{
	if (h->TX && HAL_GetTick()-h->lastTransmit>3000)
		h->TX=false;

	if (HAL_GetTick () - h->lastStatusCheck > 10000)
		{
			h->lastStatusCheck = HAL_GetTick ();
			NetStatusCheck ();
		}

	if (!h->plugged)
		return;

	if (!h->initialised && settings.SSID[0] != 0 && settings.WiFiPass[0] != 0)
		{
			NetInitialise (h);
		}

	if (h->online && !h->TX)
		{
			if (h->nMessages)
				{
					h->TX = true;
					h->lastTransmit=HAL_GetTick();
					SendCommand (UARTW_GETSTRING, h->messages[h->nMessages - 1]);
					memset(h->messages[h->nMessages - 1],0,sizeof(h->messages[h->nMessages - 1]));
					h->nMessages--;
				}
		}


}

void NetInitialise (NetHandler_t *h)
{
	SendCommand (UARTW_SSID, settings.SSID);
	HAL_Delay (100);
	SendCommand (UARTW_WIFIPASS, settings.WiFiPass);
	h->initialised = true;
}

void NetTurnOffDebug ()
{
	SendCommand (UARTW_DEBUGOFF, "0");
}

void NetSendAsync (NetHandler_t *h, char *ptr)
{
	if (h->nMessages < 7)
		{
			memcpy (h->messages[h->nMessages], ptr, strlen (ptr));
			h->nMessages++;
		}
}

void NetSendDoneCallback (NetHandler_t *h)
{

}

void NetCheckMessages (NetHandler_t *h)
{

}

void NetStatusCheck ()
{
	SendCommand (UARTW_STATUS, "");
}

void NetRefreshNetworkList (NetHandler_t *h)
{
	h->netRefreshDone = false;
	h->netRefreshInProgress=true;
	memset (h->nets, 0, sizeof(h->nets));
	SendCommand (UARTW_WIFIREFRESH, "");
}
