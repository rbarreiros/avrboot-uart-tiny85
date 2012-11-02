#ifndef _COMMON_H_
#define _COMMON_H_

#ifndef F_CPU
#define F_CPU 8000000
#endif

// RX RX Pin
#define UART_TXD PB1
#define UART_RXD PB0
#define UART_RX_PCINT 0

// Baudrate
#define BAUDRATE 9600L

// Time to wait in seconds for a uart comms to enter boot mode
// if this time passes, we go to main program.
#define WAIT 5

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef signed char bool;
typedef void(*FUNCPTR)(void);

#endif
