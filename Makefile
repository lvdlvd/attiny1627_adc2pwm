OBJS=main.o
ELF=$(notdir $(CURDIR)).elf  
HEX=$(notdir $(CURDIR)).hex
F_CPU=20000000L

ATDIR=../Atmel.ATtiny_DFP.2.0.368


CFLAGS=-mmcu=attiny1627 -B $(ATDIR)/gcc/dev/attiny1627/ -O3
CFLAGS+=-I $(ATDIR)/include/ -DF_CPU=$(F_CPU)
LDFLAGS=-mmcu=attiny1627 -B$(ATDIR)/gcc/dev/attiny1627/
CC=avr-gcc
LD=avr-gcc

all:    $(HEX)  

$(ELF): $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

$(HEX): $(ELF)
	avr-objcopy -O ihex -R .eeprom $< $@

flash:  $(HEX)
	pyupdi -d tiny1627 -c /dev/tty.usbserial-FTF5HUAV -f $(HEX)

read-fuses:
	pyupdi -d tiny1627 -c /dev/tty.usbserial-FTF5HUAV -fr

clean:
	rm -rf $(OBJS) $(ELF) $(HEX)

