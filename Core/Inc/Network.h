#include "main.h"

typedef struct NetHandler_s
{
bool plugged;
bool initialised;
bool connected;
bool online;
bool serviceAvailable;
bool TX;
bool netRefreshInProgress;
bool netRefreshDone;
uint8_t currentMessage;
uint32_t lastTransmit;
uint32_t lastStatusCheck;
char ip[24]; //Local IP
int rssi; //Signal level
char nets[16][32]; //Available WiFi Networks
char messages[8][128]; //Queue of messages
uint8_t nMessages;
uint8_t netCount;
char response[256];
}NetHandler_t;

void NetRoutine (NetHandler_t* h);
void NetInitialise (NetHandler_t* h);
void NetSendAsync (NetHandler_t* h, char *ptr);
void NetSendDoneCallback (NetHandler_t* h);
void NetCheckMessages (NetHandler_t* h);
void NetRefreshNetworkList (NetHandler_t* h);
void NetTurnOffDebug ();
void NetStatusCheck();
