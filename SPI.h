/* 
 * File:  SPI.h 
 * Author: Tom Grubb
 * Comments:
 * Revision history: 
 */

#ifndef SPI_H
#define	SPI_H

// SRAM Function Macros

#ifndef SRAM_CS             // CS macro for SRAM
#define SRAM_CS 0x01
#endif

#ifndef SRAM_WRITE          // WRITE instruction macro
#define SRAM_WRITE 0x02
#endif

#ifndef SRAM_READ           // READ instruction macro
#define SRAM_READ 0x03
#endif

#ifndef SRAM_RDMR           // READ MODE REG instruction macro
#define SRAM_RDMR 0x05
#endif

#ifndef SRAM_WRMR           // WRITE MODE REG instruction macro
#define SRAM_WRMR 0x01
#endif

// CODEC ADDRESSSES
#ifndef M_CTRL_1
#define M_CTRL_1 (uint8_t)0x01
#endif

#ifndef DAC_CTRL
#define DAC_CTRL (uint8_t)0x02
#endif

#ifndef DAC_MIX
#define DAC_MIX (uint8_t)0x03
#endif

#ifndef DAC_A_VOL
#define DAC_A_VOL (uint8_t)0x04
#endif

#ifndef DAC_B_VOL
#define DAC_B_VOL (uint8_t)0x05
#endif

#ifndef ADC_CTRL
#define ADC_CTRL (uint8_t)0x06
#endif

#ifndef M_CTRL_2
#define M_CTRL_2 (uint8_t)0x07
#endif

void SPI_Codec_Config(void);
void SPI_init(void);
void SPI_SRAM_Config(void);
void SPI_SRAM_Write(int CS, unsigned long int reg_add, int data);
int SPI_SRAM_Read(int CS, unsigned long int reg_add);
void SPI_SRAM_BlockWrite(int CS, unsigned long int reg_add, int data[]);
void SPI_SRAM_BlockRead(int CS, unsigned long int reg_add, int value[], int elements);


#endif      /*    SPI.H    */


