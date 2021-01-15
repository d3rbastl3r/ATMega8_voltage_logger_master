PROJECT=atmega8_voltage_logger_master
PARTNO=m8
MCU=atmega8
BITCLOCK=32 # Adjust the Frequency of the Programmer. Use 1 on >=4MHz, 32 >=128KHz
PORT=usb
PROGRAMMER_ID=avrisp2

AVRDUDE_FLAGS=-p $(PARTNO) -c $(PROGRAMMER_ID) -P $(PORT) -B $(BITCLOCK)

$(PROJECT).hex: ./obj/main.o ./obj/SimpleDataBuffer.o
	avr-gcc ./obj/main.o ./obj/SimpleDataBuffer.o -mmcu=$(MCU) -Os -o $(PROJECT).hex
	avr-size -B $(PROJECT).hex

./obj/main.o: ./src/main.cpp
	avr-gcc -c ./src/main.cpp -mmcu=$(MCU) -Os -o ./obj/main.o

./obj/SimpleDataBuffer.o: ./src/d3rbastl3r/SimpleDataBuffer.cpp
	avr-gcc -c ./src/d3rbastl3r/SimpleDataBuffer.cpp -mmcu=$(MCU) -Os -o ./obj/SimpleDataBuffer.o

clean:
	rm ./obj/*.o $(PROJECT).hex

flash:
	avrdude $(AVRDUDE_FLAGS) -U flash:w:$(PROJECT).hex

read_eeprom:
	avrdude $(AVRDUDE_FLAGS) -U eeprom:r:$(PROJECT).eep:r

write_eeprom:
	avrdude $(AVRDUDE_FLAGS) -U eeprom:w:$(PROJECT).eep:r

read_fuses:
	avrdude $(AVRDUDE_FLAGS) -U hfuse:r:$(PROJECT).hfuse:r -U lfuse:r:$(PROJECT).lfuse:r

write_fuses:
	avrdude $(AVRDUDE_FLAGS) -U hfuse:w:$(PROJECT).hfuse:r -U lfuse:w:$(PROJECT).lfuse:r