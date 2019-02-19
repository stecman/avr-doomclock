DEVICE     = attiny13a
CLOCK      = 9600000
PROGRAMMER = -c dragon_isp -B 125kHz
SOURCES    = main.c nmea.c softuart.c
OBJECTS    = $(SOURCES:.c=.o)

AVRDUDE = avrdude $(PROGRAMMER) -p t13
CFLAGS = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)
CFLAGS += -I -I. -I./lib/
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections
CFLAGS += -Wl,--relax -mcall-prologues
CFLAGS += -std=gnu11

.PHONY: test

# symbolic targets:
all: $(SOURCES) main.hex

.c.o:
	$(CFLAGS) -c $< -o $@

.S.o:
	$(CFLAGS) -x assembler-with-cpp -c $< -o $@
	# "-x assembler-with-cpp" should not be necessary since this is the default
	# file type for the .S (with capital S) extension. However, upper case
	# characters are not always preserved on Windows. To ensure WinAVR
	# compatibility define the file type manually.

.c.s:
	$(CFLAGS) -S $< -o $@

test:
	$(MAKE) --no-print-directory -C test

flash: all test
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	@echo "  Fuse with -U lfuse:w:0x7a:m -U hfuse:w:0xf9:m"
	@echo "  (default setting with CLKDIV8 unticked)"
	@echo "  For computing fuse byte values see the fuse bit calculator at http://www.engbedded.com/fusecalc/"

clean:
	find -name '*.d' -exec rm {} +
	find -name '*.o' -exec rm {} +
	rm -f main.hex main.elf

	$(MAKE) --no-print-directory -C test clean

# file targets:
main.elf: $(OBJECTS)
	$(CFLAGS) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=$(DEVICE) main.elf
	# If you have an EEPROM section, you must also create a hex file for the
	# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d main.elf
