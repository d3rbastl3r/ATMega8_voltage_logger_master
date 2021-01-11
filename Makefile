PROJECT=atmega8_voltage_logger_master
PARTNO=m8
MCU=atmega8
BITCLOCK=32 # Adjust the Frequency of the Programmer. Use 1 on >=4MHz, 32 >=128KHz
PORT=usb
PROGRAMMER_ID=avrisp2

AVRDUDE_FLAGS=-p $(PARTNO) -c $(PROGRAMMER_ID) -P $(PORT) -B $(BITCLOCK)

$(PROJECT).hex: src/main.cpp
	avr-gcc src/main.cpp -mmcu=$(MCU) -Os -o $(PROJECT).hex

hex_w:
	avrdude $(AVRDUDE_FLAGS) -U flash:w:$(PROJECT).hex

eep_r:
	avrdude $(AVRDUDE_FLAGS) -U eeprom:r:$(PROJECT).eep:r

eep_w:
	avrdude $(AVRDUDE_FLAGS) -U eeprom:w:$(PROJECT).eep:r

fuse_r:
	avrdude $(AVRDUDE_FLAGS) -U hfuse:r:$(PROJECT).hfuse:r -U lfuse:r:$(PROJECT).lfuse:r

fuse_w:
	avrdude $(AVRDUDE_FLAGS) -U hfuse:w:$(PROJECT).hfuse:r -U lfuse:w:$(PROJECT).lfuse:r