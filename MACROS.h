#ifndef MACROS_H
#define	MACROS_H

#define Fosc 80000000                       // Primary w/ PLL oscillator (Hz)
#define FCY (unsigned long)(Fosc/2)         // instruction clock (Hz)
#define Fs 48000                            // CODEC sampling freq (Hz)

#define EEPROM_ADD         0b1010101        // I2C EEPROM Device Address
#define CODEC_ADD (uint8_t)0b0010000        // CODEC I2C Device Address

#define FRAME 256                           // Size of RX and TX buffers
#define BLOCK 2*FRAME                     // Number of SRAM transfers per buffer
#define maxPtr (unsigned long)130048        // 1FFFF => 131071 - (2*FRAME) = 130559
#define wrapBack (unsigned long) (maxPtr - (BLOCK))
#define boundary (unsigned long) (maxPtr + (BLOCK))

// SRAM Chip Select Outputs
#define CS0 LATBbits.LATB3
#define CS1 LATBbits.LATB5
#define CS2 LATAbits.LATA4
#define CS3 LATBbits.LATB4

#endif /* MACROS_H */

