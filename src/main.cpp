#ifndef __AVR_ATmega8__
    #define __AVR_ATmega8__
#endif

#define F_CPU 1843200UL

#define BAUD 9600
#define HC05_UBRR ((F_CPU/16/BAUD)-1)

#include <avr/io.h>
#include <avr/interrupt.h> 
#include <avr/sleep.h>

#include <util/delay.h>
#include <util/twi.h>

// TWI
#define TWI_SLAVE_ADDR 0x04
// TWI

#define TX_BUFFER_SIZE 32

#define COMMAND_PARAM_BUFFER_SIZE 2

#define COMMAND_READ_VOLTAGE 0x01

#define DEBUG_COMMAND_LED_ON 0xD1
#define DEBUG_COMMAND_LED_OFF 0xD0

#define COMMAND_DO_TWI_READ 0x03
#define COMMAND_DO_TWI_WRITE 0x04

struct TXBuffer {
    volatile uint8_t buffer[TX_BUFFER_SIZE];
    volatile uint8_t posRead;
    volatile uint8_t posWrite;
    volatile bool isReady; // true if the buffer is ready to transmit

    TXBuffer():posRead(0),posWrite(0),isReady(false){}
} txBuffer;

volatile uint8_t command = 0;
volatile uint8_t commandParam[COMMAND_PARAM_BUFFER_SIZE];
volatile uint8_t commandParamWPos = 0; // Write position for command parameter

void debugLedON();
void debugLedOFF();

void handleCommands();
void handleTWIWriteCommand();
void handleTWIReadCommand();
// void handleReadVoltageCommand();
void handleUnknownCommand();

void append2TxBuffer(uint8_t);
void append2TxBufferAsStr(uint16_t);
void transmitFromTxBuffer();


void initSPIMaster() {
    /* Set MOSI and SCK output, all others input */
    DDRB |= (1<<DDB3)|(1<<DDB5);

    /* Enable SPI, Master, set clock rate fck/128 */
    SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);

    DDRC |= (1<<DDC5); // Slave Wake Up Pin set to output
    PORTC |= (1<<PC5); // Turn pin HIGH per default. On LOW slave will wake up
}

// ### Configuration for HC-05: (Baud Rate 9600 with 8 data bits, no parity and 1 stop bit)
/**
 * Default configuration on HC-05 module in "data mode"
 * `-> Type: Slave
 * `-> Baud Rate 9600 with 8 data bits, no parity and 1 stop bit
 */
void initUSART() {
    // Set baud rate
    UBRRH = (HC05_UBRR>>8);
    UBRRL = HC05_UBRR;

    DDRD |= (1<<DDD1);

    // RXCIE | RX Complete Interrupt Enable
    // TXCIE | TX Complete Interrupt Enable
    // RXEN  | Receiver Enable
    // TXEN  | Transmitter Enable
    UCSRB |= (1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN);

    // UPM[1:0]  - 0b00  | Parity Mode Disabled
    // USBS      - 0b0   | 1 Stop Bit
    // UCSZ[2:0] - 0b011 | 8-bit Character Size
    UCSRC |= (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
}

/**
 * Two-wire Serial Interface (Page 159)
 */
void initTWIMaster() {
    // On F_CPU 1.843.200 for Baud 9600 set Prescaler = 1 and TWBR = 12 (Formula on Page 160)
    // TWPS[1:0] - 0b00         | Set Prescaler value to 1 (Page 184)
    // TWSR |= (0<<TWPS1) | (0<<TWPS0)
    
    // TWBR[7:0] - 0b00001100   | Set bit rate devision factor to 12
    TWBR = 0b00001100;

    // TWCD Described on Page 183
    // TWIE - 0b1   | Enable the TWI Interrupt (Global interrupt is enabled)
    TWCR |= (1<<TWIE);

    // Activate internal pull-up resistors on SCL and SDA
    // The internal pull-ups can in some systems eliminate the need for external ones.
    PORTC |= (1<<PC4);
    PORTC |= (1<<PC5);
}

void init() {
    cli();

    initUSART();
    initSPIMaster();
    initTWIMaster();

     DDRB |= (1<<DDB0);  // LED on / off for debugging

    sei();
}

int main(void) {
    init();

    while (1) {
        handleCommands();

        // UART | Start / Init transmit data if available
        if (UCSRA & (1<<UDRE)) { // If buffer is empty
            if (txBuffer.isReady) {
                txBuffer.isReady = false;
                transmitFromTxBuffer();
            }
        }

        // Reset tx buffer if transmission is completed
        if (txBuffer.posRead != 0 && txBuffer.posRead >= txBuffer.posWrite) {
            txBuffer.posRead = 0;
            txBuffer.posWrite = 0;
        }
    }

    return 0;
}


//  _____   _    _   _____            _____ _____ ___  ______ _____ 
// |_   _| | |  | | |_   _|          /  ___|_   _/ _ \ | ___ \_   _|
//   | |   | |  | |   | |    ______  \ `--.  | |/ /_\ \| |_/ / | |  
//   | |   | |/\| |   | |   |______|  `--. \ | ||  _  ||    /  | |  
//   | |   \  /\  /  _| |_           /\__/ / | || | | || |\ \  | |  
//   \_/    \/  \/   \___/           \____/  \_/\_| |_/\_| \_| \_/  

void twiSendSTARTCondition() {TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);}
void twiSendSTOPCondition() {TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);}

void twiHandleError() {debugLedON();}

//  _____   _    _   _____            _____ _____ ___________ 
// |_   _| | |  | | |_   _|          /  ___|_   _|  _  | ___ \
//   | |   | |  | |   | |    ______  \ `--.  | | | | | | |_/ /
//   | |   | |/\| |   | |   |______|  `--. \ | | | | | |  __/ 
//   | |   \  /\  /  _| |_           /\__/ / | | \ \_/ / |    
//   \_/    \/  \/   \___/           \____/  \_/  \___/\_| 

/**
 * Handle commands incoming via UART if available
 */
void handleCommands() {
    switch(command) {
        case 0x00: // Skip NULL character
        case 0x0A: // Skip new line character
        case 0x0D: // Skip new line character
        break;

        case DEBUG_COMMAND_LED_ON:
            debugLedON();
        break;

        case DEBUG_COMMAND_LED_OFF:
            debugLedOFF();
        break;

        case COMMAND_DO_TWI_WRITE:
            handleTWIWriteCommand();
        break;

        case COMMAND_DO_TWI_READ:
            handleTWIReadCommand();
        break;

        // case COMMAND_READ_VOLTAGE:
        //     handleReadVoltageCommand();
        // break;

        default:
            handleUnknownCommand();
    }

    // Reset command
    command = 0;
    commandParamWPos = 0;
}

void handleTWIWriteCommand() {
    twiSendSTARTCondition();
    while (!(TWCR & (1<<TWINT))); // Wait for TWINT Flag set. This indicates that the START condition has been transmitted
    if ((TWSR & TW_STATUS_MASK) != TW_START) { // Check value of TWI Status Register. Mask prescaler bits. If status different from START go to ERROR
        twiHandleError();
        return;
    }

    TWDR = (TWI_SLAVE_ADDR<<1) | TW_WRITE; // Load SLA_W into TWDR Register.
    TWCR = (1<<TWINT) | (1<<TWEN); // Clear TWINT bit in TWCR to start transmission of address
    while (!(TWCR & (1<<TWINT))); // Wait for TWINT Flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
    if ((TWSR & TW_STATUS_MASK) != TW_MT_SLA_ACK) { // Check value of TWI Status Register. Mask prescaler bits. If status different from MT_SLA_ACK go to ERROR
        twiHandleError();
        return;
    }

    TWDR = 'L'; // Load DATA into TWDR Register.
    TWCR = (1<<TWINT) | (1<<TWEN); // Clear TWINT bit in TWCR to start transmission of data
    while (!(TWCR & (1<<TWINT))); // Wait for TWINT Flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received.
    if ((TWSR & TW_STATUS_MASK) != TW_MT_DATA_ACK) { // Check value of TWI Status Register. Mask prescaler bits. If status different from MT_DATA_ACK go to ERROR
        twiHandleError();
        return;
    }

    twiSendSTOPCondition();
}

void handleTWIReadCommand() {

}

/**
 * LED ON / OFF for Debugging
 */
void debugLedON() {PORTB |= (1<<PB0);}
void debugLedOFF() {PORTB &= ~(1<<PB0);}

/**
 * Handle command to read cell voltage
 *
 * Expected command params: 0
 */
// void handleReadVoltageCommand() {
//     uint8_t voltageData[] = {0, 0};

//     /* Trigger INT0 */
//     PORTC &= ~(1<<PC5); // Wake Up Slave
//     _delay_ms(250); // Gieve a little time for Slave
//     PORTC |= (1<<PC5); // Turn PIN HIGH again
//     _delay_ms(250); // Gieve a little time for Slave

//     /* SPI */
//     SPDR = 0x33;
//     while(!(SPSR & (1<<SPIF)));
//     voltageData[1] = SPDR;

//     _delay_ms(20); // Gieve a little time for Slave

//     SPDR = 0x44;
//     while(!(SPSR & (1<<SPIF)));
//     voltageData[0] = SPDR;

//     _delay_ms(20); // Gieve a little time for Slave

//     append2TxBuffer('[');
//     append2TxBuffer(command);
//     append2TxBuffer(']');
//     append2TxBuffer(':');
//     append2TxBuffer(' ');
//     append2TxBufferAsStr(voltageData[0]|voltageData[1]<<8);
//     append2TxBuffer('\n');
//     txBuffer.isReady = true; // Ready to print out

//     // Reset command
//     command = 0;
//     commandParamWPos = 0;
// }

void handleUnknownCommand() {
    append2TxBuffer('u');
    append2TxBuffer('n');
    append2TxBuffer('k');
    append2TxBuffer('n');
    append2TxBuffer('o');
    append2TxBuffer('w');
    append2TxBuffer('n');
    append2TxBuffer(':');
    append2TxBuffer('[');
    append2TxBuffer(command);
    append2TxBuffer(']');
    append2TxBuffer('\n');
    txBuffer.isReady = true; // Ready to print out
}

/**
 * Add byte to tx buffer. Ignore incoming data, if buffer is full
 */
void append2TxBuffer(uint8_t data) {
    // If tx buffer is full, data will be ignored
    if (txBuffer.posWrite >= TX_BUFFER_SIZE) {
        return;
    }

    txBuffer.buffer[txBuffer.posWrite++] = data;
}

/**
 * Manual converter from uint16_t to char array and add this to the tx buffer.
 */
void append2TxBufferAsStr(uint16_t number) {
    uint16_t devider = 10000;

    // find the right size of the devider
    while ((number / devider) == 0 && devider >=  10) {
        devider /= 10;
    }

    uint16_t tempNumber = number;
    uint8_t numberPos = 0;
    for (; devider > 0; devider /= 10) {
        numberPos = tempNumber / devider;
        append2TxBuffer('0'+numberPos);
        tempNumber -= numberPos * devider;
    }
}

/**
 * UART
 * Transmitting the next available byte via UART
 */
void transmitFromTxBuffer() {
    // If tx read pos is at the end of the buffer, nothing there to transmit
    if (txBuffer.posRead >= TX_BUFFER_SIZE) {
        return;
    }

    // Transmit if we have still data in buffer
    if (txBuffer.posRead < txBuffer.posWrite) {
        UDR = txBuffer.buffer[txBuffer.posRead++];
    }
}

/**
 * 2-wire Serial Interface
 */
ISR(TWI_vect) {
    // TWINT - 0b1  | TWI Interrupt will be executed. This bit need to be cleared manually
    TWCR &= ~(1<<TWINT);
}

/**
 * Serial Transfer Complete
 */
ISR(SPI_STC_vect) {
}

/**
 * USART, Rx Complete
 * `-> UCSRB |= (1<<RXCIE);
 */
ISR(USART_RXC_vect) {
    if (command == 0) {
        command = UDR;
    }

    // Read command parameter
    else {
        // Read command parameter only if buffer has free space, ignore otherwise
        if (commandParamWPos < COMMAND_PARAM_BUFFER_SIZE) {
            commandParam[commandParamWPos++] = UDR;
        }
    }
}

/**
 * USART, Tx Complete
 * `-> UCSRB |= (1<<TXCIE);
 */
ISR(USART_TXC_vect) {
    transmitFromTxBuffer();
}