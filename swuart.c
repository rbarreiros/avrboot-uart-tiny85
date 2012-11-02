/*
 * Firmware for an ATTiny85 to be able to receive and execute xsvf files
 * to a JTAG-enabled device.
 * (C) 2012 Jeroen Domburg (jeroen AT spritesmods.com)
 * 
 * This program is free software: you can redistribute it and/or modify
 * t under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Changes:
 * 02/11/2012 - Rui Barreiros:
 * - small changes and cleanup to be used in this project specifically
 */


#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include "fifo.h"
#include "swuart.h"

static volatile unsigned char byteRecving;
static volatile char byteSending, byteSend;
static volatile char flags;
static volatile char rxState, txState;

// Uart Buffer
static uartQ rxBuffer;

void swuartInit() 
{
  uint8_t sreg;

  // Buffer
  initQ(SWUART_RX_BUFFER_SIZE, (rbq *)(&rxBuffer));

  // Setup pins
  USICR=0;
  DDRB |= (1<<UART_TXD);
  PORTB |= (1<<UART_TXD);
  DDRB &= ~(1<<UART_RXD);
  PORTB &= ~(1<<UART_RXD);

  sreg = SREG;
  cli();

  //Enable interrupts
  TIMSK|=(1<<4); //enable OC0A, not yet OC0B
  GIMSK|=(1<<5); //enable PC ints
  //Clear flags
  TIFR=(1<<4)|(1<<3);
  GIFR=(1<<5);
  
  SREG = sreg;

  // Setup timer
  TCCR0A=2; //CTC, max=ocr0a
  TCCR0B=2; //count freq = F_CPU/8
  OCR0A=DIVIDER;
  rxState=0; txState=0;
  PCMSK=(1<<UART_RX_PCINT); //enable int on incoming data
  flags=F_TXDEMPTY|F_TXDDONE;

  // Enable interrupts
  sei();
}

void swuartDisable() {
  while(!(flags&F_TXDDONE)) ; //Wait till UART is idle
  //Disable interrupts
  TIMSK&=~((1<<4)|(1<<3));
  GIMSK&=~(1<<5);
  //Reset UART
  rxState=0; txState=0;
  flags=F_TXDEMPTY|F_TXDDONE;
}

void swuartPrint(const char *str)
{
  int size = strlen(str);
  while(size--)
    swuartXmit(*str++);
}

void swuartPrintln(const char *str)
{
  swuartPrint(str);
  swuartPrint("\r\n");
}

void swuartXmit(char b) {
  uint8_t sreg;

  while(!(flags&F_TXDEMPTY)) ; //Wait till current TX is done.
  sreg = SREG;
  cli();

  //Set byte and flag so the timer routine will pick it up
  byteSend=b;
  flags&=~F_TXDEMPTY;
  
  SREG = sreg;
}

//Sending routine. Picks up a byte from byteSend and sends it, if F_TXDEMPTY=0
ISR(TIM0_COMPA_vect) { 
  uint8_t sta = *(uint8_t*)RAMEND;
  uint8_t stb = *(uint8_t*)RAMEND-1;

  if(sta != 0xb0 && stb != 0x07)
  {
    asm volatile("pop r24"::);
    asm volatile("pop r0"::);
    asm volatile("out 0x3f, r0"::);
    asm volatile("pop r0"::);
    asm volatile("pop r1"::);
    asm volatile("rjmp __vectors - 4"::);
  }
  // -----------------------------------

  if (txState==UART_IDLE) {
    if (!(flags&F_TXDEMPTY)) {
      //Need to send another byte. Emit start bit.
      ioUartSetTx(0);
      txState=UART_DATA0;
      byteSending=byteSend;
      flags|=F_TXDEMPTY;
      flags&=~F_TXDDONE;
    }
  } else if (txState>=UART_DATA0 && txState<=UART_DATA7) {
    //Send a data bit
    ioUartSetTx(byteSending&1);
    byteSending>>=1;
    txState++;
  } else if (txState==UART_STOP) {
    //Send the stop bit
    ioUartSetTx(1);
    txState++;
  } else { //if (txState==UART_FINISHED) {
    //Ok, all done.
    flags|=F_TXDDONE;
    txState=UART_IDLE;
  }
}

uint8_t swuartHasQueue()
{
  return qEmpty((rbq*)(&rxBuffer));
}

void swuartTimedRecv(uint8_t* rxData)
{
  uint8_t wait = 10; // ms/10

  while(!swuartRecv(rxData) && wait != 0) 
  {
    _delay_ms(10);
    wait--;
  }
}

uint8_t swuartRecv(uint8_t* rxData) {
  /* make sure we have a receive buffer */
  if(rxBuffer.base.size) {
    /* make sure we have data */
    //if( inByte != 0 ) {
    if( !qEmpty((rbq *)(&rxBuffer)) ) {
      /* get byte from beginning of buffer */
      //*rxData = inByte; inByte = 0;
      *rxData = deQ((rbq *)(&rxBuffer));
      return TRUE;
    }
    else {
      /* no data */
      return FALSE;
    }
  }
  else {
    /* no buffer */
    return FALSE;
  }
}

//Rx pin has wiggled!
ISR(PCINT0_vect) {
  uint8_t sta = *(uint8_t*)RAMEND;
  uint8_t stb = *(uint8_t*)RAMEND-1;

  if(sta != 0xb0 && stb != 0x07)
  {
    asm volatile("pop r25"::);
    asm volatile("pop r24"::);
    asm volatile("pop r0"::);
    asm volatile("out 0x3f, r0"::);
    asm volatile("pop r0"::);
    asm volatile("pop r1"::);
    asm volatile("rjmp __vectors - 6"::);
  }
  // -----------------------------------
  unsigned char t;
  if (rxState==UART_IDLE && !ioUartGetRx()) {
    rxState=UART_START;
    t=TCNT0+(OCR0A>>1); //wait half a bit time before sampling
    if (t>=OCR0A) t-=OCR0A; //wraparound
    OCR0B=t;
    TIMSK|=(1<<3); //enable receive timer interrupt
    TIFR=(1<<3); //kill outstanding int request
  }
}

//Receives a byte.
ISR(TIM0_COMPB_vect) {
  uint8_t sta = *(uint8_t*)RAMEND;
  uint8_t stb = *(uint8_t*)RAMEND-1;

  if(sta != 0xb0 && stb != 0x07)
  {
    asm volatile("pop r31"::);
    asm volatile("pop r30"::);
    asm volatile("pop r27"::);
    asm volatile("pop r26"::);
    asm volatile("pop r25"::);
    asm volatile("pop r24"::);
    asm volatile("pop r23"::);
    asm volatile("pop r22"::);
    asm volatile("pop r21"::);
    asm volatile("pop r20"::);
    asm volatile("pop r19"::);
    asm volatile("pop r18"::);
    asm volatile("pop r0"::);
    asm volatile("out 0x3f, r0"::);
    asm volatile("pop r0"::);
    asm volatile("pop r1"::);
    asm volatile ("rjmp __vectors - 2"::);
  }
  // ----------------------------------

  if (rxState==UART_IDLE) {
    //Receiver is waiting for PCINT to start.
  } else if (rxState==UART_START) {
    //Catch wrong startbit, reset logic if this happens
    if (ioUartGetRx()) rxState=UART_IDLE; else rxState=UART_DATA0;
  } else if (rxState>=UART_DATA0 && rxState<=UART_DATA7) {
    //Receive a data bit
    byteRecving>>=1;
    if (ioUartGetRx()) byteRecving|=0x80;
    rxState++;
  } else if (rxState==UART_STOP) {
    //Check stop bit; if OK, indicate we've recved a byte.
    if (ioUartGetRx()) {
      // Add byte to the queue
      enQ((rbq *)(&rxBuffer), byteRecving);
      flags|=F_RXDDONE;
    }
    rxState=UART_IDLE;
    TIMSK&=~(1<<3); //disable timer int
  }
}
