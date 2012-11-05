/*
 * Software Uart Bootloader for attiny85 compatible with AVRPROG (butterfly)
 * working with avrdude.
 *
 * To boot into the bootloader, reset, and the MCU waits for 5 seconds for
 * uart communication. Upon receiving anything will enter boot mode.
 * After avrdude exits or finishes programming it will return to main application.
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
 * This software is highly based on micronucleus-t85 and 
 * AVR UART Bootloader from Martin Thomas.
 * 
 * Changes:
 * 02-10-2012 - Rui Barreiros
 * - Implementation, lot's of glueing around and fixes
 * 05-10-2012 - Rui Barreiros
 * - Proper watchdog disable at boot start
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include "swuart.h"

/*
 CHECK MAKEFILE
# hexadecimal address for bootloader section to begin. To calculate the best value:
# - make clean; make main.hex; ### output will list data: 2124 (or something like that)
# - for the size of your device (8kb = 1024 * 8 = 8192) subtract above value 2124... = 6068
# - How many pages in is that? 6068 / 64 (tiny85 page size in bytes) = 94.8125
# - round that down to 94 - our new bootloader address is 94 * 64 = 6016, in hex = 1780
*/
// verify the bootloader address aligns with page size
#if BOOTLOADER_ADDRESS % SPM_PAGESIZE != 0
#  error "BOOTLOADER_ADDRESS in makefile must be a multiple of chip's pagesize"
#endif

// Boot loader device type
#define DEVTYPE 0x21

// Bootloader version
#define VERSION_HIGH '0'
#define VERSION_LOW '1'

// Block buffer
uint8_t gBuffer[SPM_PAGESIZE];
// data for tinyvector table
static uint16_t vectorTemp[4];

#if (SPM_PAGESIZE > UINT8_MAX)
typedef uint16_t pagebuf_t;
#else
typedef uint8_t pagebuf_t;
#endif

// Vector table offsets
#define RESET_VECTOR_OFFSET 0x00       // points to our bootloader reset vector
#define PCINT0_VECTOR_OFFSET 0x02      // points to our bootloader pin change vector
#define TIM0_COMPA_VECTOR_OFFSET 0x0A  // points to our bootloader tim0 comp a vector
#define TIM0_COMPB_VECTOR_OFFSET 0x0B  // points to our bootloader tim0 comp b vector

// Our own vector table offset
#define TINY_RESET_VECTOR_OFFSET 8       // points to application reset vector
#define TINY_PCINT0_VECTOR_OFFSET 6      // points to application pcint0 vector
#define TINY_TIM0_COMPA_VECTOR_OFFSET 4  // points to timer 0 comp a vector
#define TINY_TIM0_COMPB_VECTOR_OFFSET 2  // points to timer 0 comp b vector

// Declare our functions
static inline uint16_t readEEpromPage(uint16_t address, pagebuf_t size);
static inline uint16_t writeEEpromPage(uint16_t address, pagebuf_t size);
static inline uint16_t readFlashPage(uint16_t waddr, pagebuf_t size);
static inline uint16_t writeFlashPage(uint16_t address, pagebuf_t size);
static void updateTinyTable();
static inline void eraseApplication();
static inline void leaveBootloader();
static inline void recvBuffer(pagebuf_t size);
static void fixVectors();

/*****************************************
   Implementation
******************************************/

void PushMagicWord (void) __attribute__ ((naked)) __attribute__ ((section (".init3")));

// put the word "B007" at the bottom of the stack (RAMEND - RAMEND-1)
void PushMagicWord (void) {
  asm volatile("ldi r16, 0xB0"::);
  asm volatile("push r16"::);
  asm volatile("ldi r16, 0x07"::);
  asm volatile("push r16"::);
}

static void fixVectors()
{
  uint16_t rjmp;
  uint8_t i;
  uint8_t sreg;
  uint8_t vectors[4] = {
    RESET_VECTOR_OFFSET*2,
    PCINT0_VECTOR_OFFSET*2,
    TIM0_COMPA_VECTOR_OFFSET*2,
    TIM0_COMPB_VECTOR_OFFSET*2,
  };

  rjmp = pgm_read_word(BOOTLOADER_ADDRESS) + BOOTLOADER_ADDRESS / 2;
  if(rjmp != pgm_read_word(0))
  {
    sreg = SREG;
    cli();

    for( i = 0; i < 4; i++)
    {
      rjmp = pgm_read_word(BOOTLOADER_ADDRESS + vectors[i]) + BOOTLOADER_ADDRESS / 2;
      boot_page_fill(vectors[i], rjmp);
    }

    eeprom_busy_wait();

    boot_page_erase(0x00);
    boot_spm_busy_wait();
    boot_page_write(0x00);
    boot_spm_busy_wait();

    SREG = sreg;
  }
}


/**
 * Reads eeprom address and sends it through uart
 *
 * @param address The starting memory address to read
 * @param size The size of memory to be read
 * @return Returns the address+size (next address to be read)
 */
static inline uint16_t readEEpromPage(uint16_t address, pagebuf_t size)
{
  do {
    swuartXmit( eeprom_read_byte( (uint8_t*)address ) );
    address++;
    size--;
  } while (size);

  return address;
}

/**
 * Writes eeprom page
 *
 * @param starting address to write data on to
 * @param size of bytes to write
 * @return returns the address+size (which is next address to be written into)
 */
static inline uint16_t writeEEpromPage(uint16_t address, pagebuf_t size)
{
  uint8_t *tmp = gBuffer;

  do {
    eeprom_write_byte( (uint8_t*)address, *tmp++ );
    address++;                      // Select next byte
    size--;                         // Decrease number of bytes to write
  } while (size);                   // Loop until all bytes written
  eeprom_busy_wait();

  return address;
}

/**
 * Fills our buffer with data from UART
 *
 * @param size Amount of data to be buffered ( < buffersize)
 */
static inline void recvBuffer(pagebuf_t size)
{
  pagebuf_t cnt;
  uint8_t *tmp = gBuffer;
  uint8_t data;

  for (cnt = 0; cnt < sizeof(gBuffer); cnt++) {
    if(cnt < size)
    {
      swuartTimedRecv(&data);
    } else 
      data = 0xFF;

    *tmp++ = data;
  }
}

/**
 * Leaves the bootloader and jumps into the main app start address
 *
 */

static inline void leaveBootloader(void)
{
  // Clear all interrupts etc
  cli();
  TCCR0A = 0;
  TCCR0B = 0;
  GIMSK = 0;
  TIMSK = 0;

  // clear magic word from bottom of stack before jumping to the app
  *(uint8_t*)(RAMEND) = 0x00;
  *(uint8_t*)(RAMEND-1) = 0x00;

  // rjump to our application address
  asm volatile ("rjmp __vectors - 8"::);
}

/**
 * Erases the flash memory between the start of the flash until
 * the bootloader start.
 *
 *
 */

static inline void eraseApplication()
{
  uint16_t address = BOOTLOADER_ADDRESS;
  uint8_t sreg;

  sreg = SREG;
  cli();

  while(address)
  {
    address -= SPM_PAGESIZE;
    boot_page_erase(address);
    boot_spm_busy_wait();
  }

  fixVectors();
  SREG = sreg;
}

/**
 * Reads flash address and sends it through uart
 *
 * Note: does not avoid the bootloader area to be read.
 *
 * @param address The starting address to be read
 * @param size The size of memory to be read
 * @return Returns the address+size (next address to be read)
 */
static inline uint16_t readFlashPage(uint16_t waddr, pagebuf_t size)
{
  uint32_t address = (uint32_t)waddr<<1;
  uint16_t data;

  do {
#if defined(RAMPZ)
      data = pgm_read_word_far(address);
#else
      data = pgm_read_word_near(address);
#endif

      // This is to make avrdude not complaint about
      // errors when reading the interrupt table area
      // after writing the flash
      if(address == RESET_VECTOR_OFFSET * 2)
        data = vectorTemp[0];
      else if(address == PCINT0_VECTOR_OFFSET * 2)
        data = vectorTemp[1];
      else if(address == TIM0_COMPA_VECTOR_OFFSET * 2)
        data = vectorTemp[2];
      else if(address == TIM0_COMPB_VECTOR_OFFSET * 2)
        data = vectorTemp[3];
      
      swuartXmit(data);                 // send LSB
      swuartXmit((data >> 8));          // send MSB
      address += 2;                       // Select next word in memory
      size -= 2;                        // Subtract two bytes from number of bytes to read
  } while (size);                     // Repeat until block has been read

  return address>>1;
}

/**
 * This does the flash writing.
 * The interrupt vectors we use are saved in a table
 * to be updated later one on our own vector table
 *
 * 
 */

static inline uint16_t writeFlashPage(uint16_t address, pagebuf_t size)
{
  uint16_t pagestart = address;
  uint16_t data, sreg;
  uint8_t *tmp = gBuffer;

  do {
    // Safeguard
    if(address >= BOOTLOADER_ADDRESS)
      return address;

    data = *tmp++;
    data |= *tmp++ << 8;

    if(address == RESET_VECTOR_OFFSET * 2)
      vectorTemp[0] = (short)data;
    else if(address == PCINT0_VECTOR_OFFSET * 2)
      vectorTemp[1] = (short)data;
    else if(address == TIM0_COMPA_VECTOR_OFFSET * 2)
      vectorTemp[2] = (short)data;
    else if(address == TIM0_COMPB_VECTOR_OFFSET * 2)
      vectorTemp[3] = (short)data;
    else
    {
      sreg = SREG;
      cli();
      boot_page_fill(address, data);
      SREG = sreg;
    }

    address += 2;
    size -= 2;
  } while (size);

  sreg = SREG;
  cli();
  boot_page_write(pagestart);
  boot_spm_busy_wait();
  SREG = sreg;

  if(pagestart == 0)
    updateTinyTable();

  return address;
}

/**
 * Update our bootloader vector table
 *
 *
 */

static void updateTinyTable()
{
  uint8_t sreg;

  sreg = SREG;
  cli();

  boot_page_fill(BOOTLOADER_ADDRESS - TINY_RESET_VECTOR_OFFSET, 
                 vectorTemp[0] + ((FLASHEND + 1) - BOOTLOADER_ADDRESS) / 2 + 4 + RESET_VECTOR_OFFSET);
  boot_page_fill(BOOTLOADER_ADDRESS - TINY_PCINT0_VECTOR_OFFSET, 
                 vectorTemp[1] + ((FLASHEND + 1) - BOOTLOADER_ADDRESS) / 2 + 3 + PCINT0_VECTOR_OFFSET);
  boot_page_fill(BOOTLOADER_ADDRESS - TINY_TIM0_COMPA_VECTOR_OFFSET, 
                 vectorTemp[2] + ((FLASHEND + 1) - BOOTLOADER_ADDRESS) / 2 + 2 + TIM0_COMPA_VECTOR_OFFSET);
  boot_page_fill(BOOTLOADER_ADDRESS - TINY_TIM0_COMPB_VECTOR_OFFSET, 
                 vectorTemp[3] + ((FLASHEND + 1) - BOOTLOADER_ADDRESS) / 2 + 1 + TIM0_COMPB_VECTOR_OFFSET);

  boot_page_write(BOOTLOADER_ADDRESS - SPM_PAGESIZE + 1);
  boot_spm_busy_wait();

  SREG = sreg;
}

int main(void)
{
  int cnt = 0;
  uint8_t y, data, device;
  uint16_t address = 0;
  uint8_t doBoot = 0;
  pagebuf_t size;

  wdt_reset();
  MCUSR = 0;
  wdt_disable();

  fixVectors();
  swuartInit();

  for(cnt = 0; cnt < WAIT; cnt++)
  {
    if(swuartRecv(&y)) 
    { 
      doBoot = 1; 
      swuartXmit('?');
      break; 
    }
    _delay_ms(1000);
  }

  while(1)
  {
    if(!doBoot) break;

    while(!swuartRecv(&y)) { _delay_ms(10); }

    if(y == 'a') // Autoincrement
    {
      swuartXmit('Y');
    }
    else if(y == 'A') // write address
    {
      swuartTimedRecv(&data); // MSB address
      address = data;
      
      swuartTimedRecv(&data); // LSB
      address = (address << 8) | data;
      
      swuartXmit('\r');
    }
    else if(y == 'b') // buffer load support
    {
      swuartXmit('Y');
      swuartXmit((sizeof(gBuffer) >> 8) & 0xff);
      swuartXmit(sizeof(gBuffer) & 0xff);
    }
    else if(y == 'B') // start buffer load
    {
      swuartTimedRecv(&data);
      size = data << 8;
      swuartTimedRecv(&data);
      size |= data;
      swuartTimedRecv(&data);
      recvBuffer(size);
      
      if(device == DEVTYPE)
      {
        if(data == 'F')
          address = writeFlashPage(address, size);
        else if (data == 'E')
          address = writeEEpromPage(address, size);

        swuartXmit('\r');
      } else 
        swuartXmit(0);

    }
    else if(y == 'g') // block read
    {
      swuartTimedRecv(&data); // high byte of buffer size
      size = data << 8;
      swuartTimedRecv(&data); // low byte of buffer size
      size |= data;
      swuartTimedRecv(&data); // mem type

      if(data == 'F')
        address = readFlashPage(address, size);
      else
        address = readEEpromPage(address, size);
    }
    else if(y == 'e') // chip erase
    {
      if(device == DEVTYPE)
        eraseApplication();
      swuartXmit('\r');
    }
    else if(y == 'E') // exit upgrade, do reset
    {
      //wdt_enable(WDTO_250MS); // enable watchdog for chip to reset itself
      swuartXmit('\r');
      _delay_ms(1000);
      break;
    }
    else if(y == 'P') // Enter programming mode
    {
      swuartXmit('\r');
    } 
    else if(y == 'L') // Leave programing mode
    {
      swuartXmit('\r');
    }
    else if(y == 'p') // Programmer type
    {
      swuartXmit('S');
    }
    else if(y == 'F')
    {
      swuartXmit(boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS));
    }
    else if(y == 'r')
    {
      swuartXmit(boot_lock_fuse_bits_get(GET_LOCK_BITS));
    }
    else if(y == 'N')
    {
      swuartXmit(boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS));
    }
    else if(y == 'Q')
    {
      swuartXmit(boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS));
    }
    else if(y == 't') // return device type
    {
      swuartXmit(DEVTYPE);
      swuartXmit(0);
    }
    else if(y == 'x' || y == 'y') // clear and set led ignores
    {
      swuartTimedRecv(&data);
      swuartXmit('\r');
    }
    else if(y == 'T') // set device
    {
      swuartTimedRecv(&device);
      swuartXmit('\r');
    }
    else if(y == 'S') // return software identifier
    {
      swuartPrint("AVRBOOT");
    }
    else if(y == 'V') // return software version
    {
      swuartXmit(VERSION_HIGH);
      swuartXmit(VERSION_LOW);
    }
    else if(y == 'v') // return hardware version
    {
      swuartXmit(VERSION_HIGH);
      swuartXmit(VERSION_LOW);
    }
    else if(y == 's') // return sig bytes
    {
      swuartXmit(SIGNATURE_2);
      swuartXmit(SIGNATURE_1);
      swuartXmit(SIGNATURE_0);
    }
    else if(y != 0x1b) // ESC
    {
      swuartXmit('?');
    }
  }

  //swuartPrintln("Leaving boot!");
  leaveBootloader();
  // go to normal program address
  return 0;
}
