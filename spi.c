/*
 * File:   spi.c
 * Author: rahul
 *
 * Created on August 23, 2024, 10:53 AM
 */

// PIC18F8770A Configuration Bit Settings
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>

#define _XTAL_FREQ 20000000  // Define crystal frequency (20 MHz)

// Function Declarations
void SPI_Init(void);
unsigned char SPI_Transmit(unsigned char data);
unsigned char SPI_Receive(void);
unsigned int ADC_Read(unsigned char channel);
void Flash_Write(unsigned char page, unsigned char offset, unsigned char data);
unsigned char Flash_Read(unsigned char page, unsigned char offset);
void SPI_ErrorHandling(void);

// Main Function
void main(void) {
    unsigned char adc_data;
    unsigned char flash_data;

    // Initialize SPI
    SPI_Init();

    while(1) {
        // Read data from MCP3008 ADC channel 0
        adc_data = ADC_Read(0);

        // Write ADC data to AT45DB041E flash memory at page 0, offset 0
        Flash_Write(0, 0, adc_data);

        // Read back data from flash memory
        flash_data = Flash_Read(0, 0);

        __delay_ms(500); // Delay before next operation
    }
}

// SPI Initialization Function
void SPI_Init(void) {
    TRISC5 = 0; // SDO (Serial Data Out) as output
    TRISC4 = 1; // SDI (Serial Data In) as input
    TRISC3 = 0; // SCK (Serial Clock) as output
    SSPCON = 0x30; // Enable SPI Master mode, clock = Fosc/4
    SSPSTAT = 0x00; // Data transmitted on rising edge, sample in middle
}

// SPI Transmit Function
unsigned char SPI_Transmit(unsigned char data) {
    SSPBUF = data; // Load data into buffer
    while(!SSPSTATbits.BF); // Wait until transmission is complete
    return SSPBUF; // Return received data
}

// SPI Receive Function
unsigned char SPI_Receive(void) {
    return SPI_Transmit(0xFF); // Send dummy data to receive
}

// ADC Read Function (MCP3008)
unsigned int ADC_Read(unsigned char channel) {
    unsigned int adc_value = 0;

    // Select MCP3008 channel
    PORTCbits.RC0 = 0; // Chip select low (MCP3008)
    SPI_Transmit(0x01); // Start bit
    SPI_Transmit((channel << 4) | 0x80); // Send channel number
    adc_value = SPI_Transmit(0x00) & 0x03; // Read first two bits
    adc_value = (adc_value << 8) | SPI_Transmit(0x00); // Read next 8 bits
    PORTCbits.RC0 = 1; // Chip select high (MCP3008)

    return adc_value;
}

// Flash Memory Write Function (AT45DB041E)
void Flash_Write(unsigned char page, unsigned char offset, unsigned char data) {
    PORTCbits.RC1 = 0; // Chip select low (AT45DB041E)
    SPI_Transmit(0x82); // Write command
    SPI_Transmit(page); // Send page number
    SPI_Transmit(offset); // Send offset
    SPI_Transmit(data); // Send data
    PORTCbits.RC1 = 1; // Chip select high (AT45DB041E)
    __delay_ms(10); // Wait for write to complete
}

// Flash Memory Read Function (AT45DB041E)
unsigned char Flash_Read(unsigned char page, unsigned char offset) {
    unsigned char data;

    PORTCbits.RC1 = 0; // Chip select low (AT45DB041E)
    SPI_Transmit(0x03); // Read command
    SPI_Transmit(page); // Send page number
    SPI_Transmit(offset); // Send offset
    data = SPI_Receive(); // Receive data
    PORTCbits.RC1 = 1; // Chip select high (AT45DB041E)

    return data;
}

// SPI Error Handling Function
void SPI_ErrorHandling(void) {
    if(SSPCONbits.WCOL) { // Check for write collision
        SSPCONbits.WCOL = 0; // Clear write collision flag
    }
    if(SSPCONbits.SSPOV) { // Check for overflow
        SSPCONbits.SSPOV = 0; // Clear overflow flag
    }
}
