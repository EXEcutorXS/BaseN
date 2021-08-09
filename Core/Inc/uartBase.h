#ifndef UART_PROT
#define UART_PROT

#include "main.h"

//NODE COMMANDS
#define UART_FREQUENCY '1'
#define UART_SF '2'
#define UART_BW '3'
#define UART_SYNCWORD '4'
#define UART_POWER '5'
#define UART_PREAMBLE '8'
#define UART_CR '9'
#define UART_POWERALARM 'P'
#define UART_NODENUM 'n'
#define UART_CALL 'C'
#define UART_ACKNOWLEDGE 'A'
#define UART_READ 'R'
#define UART_SAVE 'S'
#define UART_WORKING_INTERVAL 'i'
#define UART_WARNING_DELAY 'q'
#define UART_SUPER_WARNING_DELAY 'w'
#define UART_WORKING_INTERVAL 'i'
#define UART_USELED	'L'

#define UARTW_WIFION       'z'     //Base<-Bridge
#define UARTW_SSID	       'Z'     //Base->Bridge
#define UARTW_WIFIPASS     'X'     //Base->Bridge
#define UARTW_WIFIREFRESH  'W'     //Base->Bridge
#define UARTW_GET          'G'     //Base->Bridge  //Returns entire HTTP response
#define UARTW_GETSTRING    'F'     //Base->Bridge  //Returns only first string of HTTP response
#define UARTW_POST         'P'     //Base->Bridge
#define UARTW_STATUS       't'     //Base->Bridge
#define UARTW_CONNECTED    'u'     //Base<-Bridge
#define UARTW_ONLINE       'o'     //Base<-Bridge
#define UARTW_INITIALISED  'f'     //Base<-Bridge
#define UARTW_WIFICOUNT    'y'     //Base<-Bridge
#define UARTW_WIFISSID     'e'     //Base<-Bridge
#define UARTW_HTTPRESPONSE 'h'     //Base<-Bridge
#define UARTW_IP           'I'     //Base<-Bridge
#define UARTW_RESPONSE     'K'     //Base<-Bridge
#define UARTW_RESPONSE1    'D'     //Base<-Bridge //Single string http response
#define UARTW_HTTPFAIL     'N'     //Base<-Bridge
#define UARTW_SETHOST      'J'     //Base->Bridge
#define UARTW_DEBUGOFF     'd'     //Base->Bridge
#define UARTW_REBOOT       'M'     //Base->Bridge
#define UARTW_RSSI         'b'     //Base<>Bridge



void readByte(void);
void uartInit();
void uartReceiveHandler();
void sendConfig(void);
void configNodeViaUart(uint8_t nodeNum);
#endif
