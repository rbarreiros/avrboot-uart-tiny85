
 This is a boot loader for attiny85 using software uart
 and compatible with avrdude by implementing the butterfly
 protocol that the avr109 programmer uses.

 As soon as the MCU is powered up or is reset, the bootloader
 waits for 5 seconds (can be changed in common.h) for any
 communication on the uart pins (also defined in common.h),
 as soon as any communication is detected, the bootloader
 starts and remains in boot mode until the mcu is reset again
 or avrdude exits from either the terminal or programming mode.

 This bootloader is heavily based in micronucleus-t85, in which
 is follows the same approach (and a lot of code also) on how
 to handle with interrupts, by making it's own interrupt table
 aside from the main mcu interrupt table to handle it's own and
 application interrupts.

 Maybe someday, me or someone else will worry more in decreasing
 the size of the bootloader (I believe it is possible), for now
 it's not my main worry as the target application I'll be using
 this bootloader on won't be that big it won't fit on an attiny85.

 To program using avrdude use the avr109 programmer, like:
 avrdude -c avr109 -p attiny85 -P /dev/ttyUSB0 -b 9600 ....

