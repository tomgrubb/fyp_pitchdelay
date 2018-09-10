#ifndef MACROS_H
#define	MACROS_H

#define Fosc 80000000                       // Primary w/ PLL oscillator (Hz)
#define FCY (unsigned long)(Fosc/2)         // instruction clock (Hz)
#define Fs 48000                            // CODEC sampling freq (Hz)

#define EEPROM_ADD         0b1010101
#define CODEC_ADD (uint8_t)0b0010000

#define FRAME 256
#define maxPtr (unsigned long)125440 // 1FFFF => 131071 - (2*FRAME) = 130559


#endif /* MAIN_H */

