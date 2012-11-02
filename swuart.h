#ifndef _SWUART_H_
#define _SWUART_H_

#include "common.h"
#include "fifo.h"

#define DIVIDER (F_CPU / (BAUDRATE * 8))

//Macros, for speed
#define ioUartSetTx(p) if (p) PORTB|=(1<<UART_TXD); else PORTB&=~(1<<UART_TXD)
#define ioUartGetRx() (PINB&(1<<UART_RXD))

#define F_RXDDONE (1<<0) //Set when a byte is received and in byteRecved
#define F_TXDDONE (1<<1) //Set when transmission is done and F_TXDEMPTY is true
#define F_TXDEMPTY (1<<2) //Set when the byte in byteSend is copied to byteSending

/*
Serial states:
- Start bit. Always low.
- 8 databits
- Stop bit. Always high.
*/

//Shared for rxState and txState
#define UART_IDLE 0
#define UART_START 1
#define UART_DATA0 2
// ...
#define UART_DATA7 9
#define UART_STOP 10
#define UART_FINISHED 10

#define SWUART_RX_BUFFER_SIZE 68
#undef SWUART_INVERT

typedef struct _uartQ {
  rbq base;
  uint8_t data[SWUART_RX_BUFFER_SIZE+1];
} uartQ;

void swuartPrint(const char *str);
void swuartPrintln(const char *str);
void swuartInit();
void swuartXmit(char b);
uint8_t swuartRecv(uint8_t* rxData);
void swuartTimedRecv(uint8_t* rxData);
void swuartDisable(void);
uint8_t swuartHasQueue();

#endif
