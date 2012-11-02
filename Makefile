
# hexadecimal address for bootloader section to begin. To calculate the best value:
# - make clean; make main.hex; ### output will list data: 2124 (or something like that)
# - for the size of your device (8kb = 1024 * 8 = 8192) subtract above value 2124... = 6068
# - How many pages in is that? 6068 / 64 (tiny85 page size in bytes) = 94.8125
# - round that down to 94 - our new bootloader address is 94 * 64 = 6016, in hex = 1780
BOOTLOADER_ADDRESS = 1500

TARGET=attiny85
F_CPU=8000000
FUSES = -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xfe:m
ISP=usbasp
PORT=/dev/usb/ttyUSB0

AVRDUDE = avrdude -c $(ISP) -p $(TARGET) -P $(PORT)
CC = avr-gcc

CFLAGS = -Wall -Os -fno-move-loop-invariants -fno-tree-scev-cprop -fno-inline-small-functions -I. -Ilibs-device
CFLAGS +=  -std=gnu99 -mmcu=$(TARGET) -DF_CPU=$(F_CPU) -DBOOTLOADER_ADDRESS=0x$(BOOTLOADER_ADDRESS)

LDFLAGS = -Wl,--relax,--gc-sections
LDFLAGS += -mmcu=$(TARGET) -Wl,-Map=main.map
LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS)

OBJECTS = fifo.o swuart.o main.o

all: main.hex

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@ -Wa,-ahls=$<.lst

flash: all
	$(AVRDUDE) -U flash:w:main.hex:i

readflash:
	$(AVRDUDE) -U flash:r:read.hex:i

fuse:
	$(AVRDUDE) $(FUSE)

clean:
	rm -f main.hex main.obj main.cof main.list main.map main.eep main.bin *.o *.lst *~

# file targets:
main.bin: $(OBJECTS)
	$(CC) $(CFLAGS) -o main.bin $(OBJECTS) $(LDFLAGS)


main.hex: main.bin
	rm -f main.hex main.eep.hex
	avr-objcopy -j .text -j .data -O ihex main.bin main.hex
	avr-size main.hex

disasm:	main.bin
	avr-objdump -d main.bin

help:
	@echo "Usage: make                same as make help"
	@echo "       make help           same as make"
	@echo "       make main.hex       create main.hex"
	@echo "       make clean          remove redundant data"
	@echo "       make disasm         disasm main"
	@echo "       make flash          upload main.hex into flash"
	@echo "       make fuses          program fuses"
	@echo "       make avrdude        test avrdude"
	@echo "Current values:"
	@echo "       TARGET=${TARGET}"
	@echo "       FUSES=${FUSES}"
	@echo "       CLOCK=$(F_CPU)"
	@echo "       ISP=${ISP}"
	@echo "       PORT=${PORT}"
