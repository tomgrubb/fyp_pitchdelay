/*
 * File:   serial_comms.c
 * Author: Tom Grubb
 *  (c) Seiger Audio 2017
 *
 * Created on December 1, 2017, 10:23 AM
 * 
 */
#include <xc.h>
#include <stdint.h>
#include <libpic30.h>
#include "MACROS.h"
#include "I2C.h"
#include "SPI.h"

#define FCY 40000000UL


/* ===========================================================================
 *
 * 
 * 
 *                      ALL SPI FUNCTIONS BELOW THIS
 * 
 * 
 * 
 * ========================================================================== */

void SPI_init(void)
{
    // Set SPI control registers
    SPI1CON1bits.DISSCK = 0;                        // Internal SPI clock
    SPI1CON1bits.DISSDO = 0;                        // SDO is controlled by module
    SPI1CON1bits.MODE16 = 0;                        // Word-wide (16 bits)
    SPI1CON1bits.CKE = 1;                           // SDO changes on HIGH->LOW transition                          
    SPI1CON1bits.SSEN = 0;                          // !!Slave Select DISABLED
    SPI1CON1bits.CKP = 0;                           // Clock idle state is LOW    
    SPI1CON1bits.MSTEN = 1;                         // Master Mode enabled
    SPI1CON1bits.SMP = 0;                           // SDI is sampled in middle of SDO time
    
    // ---> SCL = FCY/(PriPRE*SecPRE) = 10 MHz
    SPI1CON1bits.SPRE = 0b100;                      // 2:1 Second Pre-Scale    
    SPI1CON1bits.PPRE = 0x3;                        // 1:1 Primary Pre-Scale
    
    SPI1CON2 = 0x00;                                // DISABLE Frame Sync etc.               
}

void SPI_SRAM_Config(void)
{
    int temp_buff;
    uint8_t mode_data = 0b01000000;
    
    LATBbits.LATB3 = 0;                             // pull !CS low (active)
    __delay32(2);                                   // ensure SRAM is active 
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    SPI1STATbits.SPIEN = 1;                         // enable SPI Module 1
    
    temp_buff = SPI1BUF;                            // clear flags by reading buffer
    SPI1BUF = SRAM_WRMR;                            // send WRITE MODE REG command
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
//    temp_buff = SPI1BUF;                          // clear flags by reading buffer
    SPI1BUF = mode_data;                            // set data for MODE REG
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    temp_buff = SPI1BUF;                            // clear flags by reading buffer
    SPI1BUF = SRAM_RDMR;                            // send WRITE MODE REG command
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
//    temp_buff = SPI1BUF;                            // clear flags by reading buffer
    SPI1BUF = 0x0;                                  // send WRITE MODE REG command
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    temp_buff = SPI1BUF;
   
    LATBbits.LATB3 = 1;                             // pull !CS high (inactive)        
}

void SPI_SRAM_Write(int CS, unsigned long int reg_add, int data)
{
    // decompose 24 bit address into 3 bytes
    char address_HI = 0;
    char address_MID = 0;      
    char address_LO = 0;     
    
    int dataHI = (data >> 8);
    int dataLO = (data & 0xFF);
    int value = 0;
    
    address_HI = reg_add >> 16;                     // format high address (bit 17)  
    address_MID = reg_add >> 8;                     // format mid address (bits 16-8)
    address_LO = (reg_add & 0xFF);                  // format low address (bits 7-0) 
    
    if (CS == 0)
    {
        LATBbits.LATB3 = 1;                         // pull CS0 high (inactive)
    }
    else if (CS == 1)
    {
        LATBbits.LATB4 = 1;                         // pull CS1 high (inactive)  
    }
    else if (CS == 2)
    {
        LATAbits.LATA4 = 1;                         // pull CS2 high (inactive)
    } 
    else
    {
        LATBbits.LATB5 = 1;                         // pull CS3 high (inactive)
    }
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    SPI1STATbits.SPIEN = 1;                         // enable SPI Module 1
    
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = (SRAM_WRITE);                         // send WRITE instruction
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = address_HI;                           // send high byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = address_MID;                          // send mid byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = address_LO;                           // send low byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete

    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = dataHI;                               // send data to be written
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete 
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = dataLO;                               // send data to be written
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete 
    
    if (CS == 0)
    {
        LATBbits.LATB3 = 1;                         // pull CS0 high (inactive)
    }
    else if (CS == 1)
    {
        LATBbits.LATB4 = 1;                         // pull CS1 high (inactive)  
    }
    else if (CS == 2)
    {
        LATAbits.LATA4 = 1;                         // pull CS2 high (inactive)
    } 
    else
    {
        LATBbits.LATB5 = 1;                         // pull CS3 high (inactive)
    }
}

void SPI_SRAM_BlockWrite(int CS, unsigned long int reg_add, int data[])
{
    // decompose 24 bit address into 3 bytes
    char address_HI = 0;
    char address_MID = 0;      
    char address_LO = 0;     
    int dataHI = 0;
    int dataLO = 0;
    int value = 0;
    int n;
    
    address_HI = reg_add >> 16;                     // format high address (bit 17)  
    address_MID = reg_add >> 8;                     // format mid address (bits 16-8)
    address_LO = (reg_add & 0xFF);                  // format low address (bits 7-0) 
    
    if (CS == 0)
    {
        LATBbits.LATB3 = 0;                         // pull CS0 high (active)
    }
    else if (CS == 1)
    {
        LATBbits.LATB4 = 0;                         // pull CS1 high (active)  
    }
    else if (CS == 2)
    {
        LATAbits.LATA4 = 0;                         // pull CS2 high (active)
    } 
    else
    {
        LATBbits.LATB5 = 0;                         // pull CS3 high (active)
    }
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    SPI1STATbits.SPIEN = 1;                         // enable SPI Module 1
    
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = (SRAM_WRITE);                         // send WRITE instruction
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = address_HI;                           // send high byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = address_MID;                          // send mid byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = address_LO;                           // send low byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete

    // write audio samples in correct order
    for (n = 0; n < FRAME; n++)
    {
        dataHI = (data[n] >> 8);
        dataLO = (data[n] & 0xFF);

        SPI1STATbits.SPIROV = 0;                        // clear overflow bit
        value = SPI1BUF;                                // clear flags by reading buffer
        SPI1BUF = dataHI;                               // send data to be written
        while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete 

        SPI1STATbits.SPIROV = 0;                        // clear overflow bit
        value = SPI1BUF;                                // clear flags by reading buffer
        SPI1BUF = dataLO;                               // send data to be written
        while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete         
    }
    
    if (CS == 0)
    {
        LATBbits.LATB3 = 1;                         // pull CS0 high (inactive)
    }
    else if (CS == 1)
    {
        LATBbits.LATB4 = 1;                         // pull CS1 high (inactive)  
    }
    else if (CS == 2)
    {
        LATAbits.LATA4 = 1;                         // pull CS2 high (inactive)
    } 
    else
    {
        LATBbits.LATB5 = 1;                         // pull CS3 high (inactive)
    }  
}

int SPI_SRAM_Read(int CS, unsigned long int reg_add)
{
    char address_HI = 0;      // highest byte of register address
    char address_MID = 0;     // middle byte of register address
    char address_LO = 0;      // lowest bytes of register address
    int value = 0;
    int dataHI = 0;
    int dataLO = 0;
    
    address_HI = reg_add >> 16;                     // format high address (bit 17)  
    address_MID = reg_add >> 8;                     // format mid address (bits 16-8)
    address_LO = (reg_add & 0xFF);                  // format low address (bits 7-0) 

    if (CS == 0)
    {
        LATBbits.LATB3 = 0;                         // pull CS0 high (active)
    }
    else if (CS == 1)
    {
        LATBbits.LATB4 = 0;                         // pull CS1 high (active)  
    }
    else if (CS == 2)
    {
        LATAbits.LATA4 = 0;                         // pull CS2 high (active)
    } 
    else
    {
        LATBbits.LATB5 = 0;                         // pull CS3 high (active)
    }
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    SPI1STATbits.SPIEN = 1;                         // enable SPI Module 1
    
    value = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = SRAM_READ;                            // send WRITE and highest address byte
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF; 
    SPI1BUF = address_HI;                           // send high byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF; 
    SPI1BUF = address_MID;                          // send mid byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF; 
    SPI1BUF = address_LO;                           // send low byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    //SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    value = SPI1BUF;
    SPI1BUF = 0x00;                                 // load buffer with something
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete    
    dataHI = SPI1BUF;                               // store received data
    
    SPI1BUF = 0x00;                                 // load buffer with something
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    dataLO = SPI1BUF;                               // store received data
   
    if (CS == 0)
    {
        LATBbits.LATB3 = 1;                         // pull CS0 high (inactive)
    }
    else if (CS == 1)
    {
        LATBbits.LATB4 = 1;                         // pull CS1 high (inactive)  
    }
    else if (CS == 2)
    {
        LATAbits.LATA4 = 1;                         // pull CS2 high (inactive)
    } 
    else
    {
        LATBbits.LATB5 = 1;                         // pull CS3 high (inactive)
    }    
   
    value = (dataHI << 8 | dataLO);
    return value;
}


void SPI_SRAM_BlockRead(int CS, unsigned long int reg_add, int value[])
{
    char address_HI = 0;      // highest byte of register address
    char address_MID = 0;     // middle byte of register address
    char address_LO = 0;      // lowest bytes of register address
    
    int temp = 0;
    int dataHI = 0;
    int dataLO = 0;
    int n;
    
    address_HI = reg_add >> 16;                     // format high address (bit 17)  
    address_MID = reg_add >> 8;                     // format mid address (bits 16-8)
    address_LO = (reg_add & 0xFF);                  // format low address (bits 7-0) 

    if (CS == 0)
    {
        LATBbits.LATB3 = 0;                         // pull CS0 high (active)
    }
    else if (CS == 1)
    {
        LATBbits.LATB4 = 0;                         // pull CS1 high (active)  
    }
    else if (CS == 2)
    {
        LATAbits.LATA4 = 0;                         // pull CS2 high (active)
    } 
    else
    {
        LATBbits.LATB5 = 0;                         // pull CS3 high (active)
    }
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    SPI1STATbits.SPIEN = 1;                         // enable SPI Module 1
    
    temp = SPI1BUF;                                // clear flags by reading buffer
    SPI1BUF = SRAM_READ;                            // send WRITE and highest address byte
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    temp = SPI1BUF; 
    SPI1BUF = address_HI;                           // send high byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    temp = SPI1BUF; 
    SPI1BUF = address_MID;                          // send mid byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    temp = SPI1BUF; 
    SPI1BUF = address_LO;                           // send low byte of address 
    while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
    
    //SPI1STATbits.SPIROV = 0;                        // clear overflow bit
    for (n = 0; n < FRAME; n++)
    {
        temp = SPI1BUF;
        SPI1BUF = 0x00;                                 // load buffer with something
        while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete    
        dataHI = SPI1BUF;                               // store received data

        SPI1BUF = 0x00;                                 // load buffer with something
        while (SPI1STATbits.SPIRBF == 0) {};            // wait until TX complete
        dataLO = SPI1BUF;                               // store received data
        
        value[n] = (dataHI << 8 | dataLO);
    }
   
    if (CS == 0)
    {
        LATBbits.LATB3 = 1;                         // pull CS0 high (inactive)
    }
    else if (CS == 1)
    {
        LATBbits.LATB4 = 1;                         // pull CS1 high (inactive)  
    }
    else if (CS == 2)
    {
        LATAbits.LATA4 = 1;                         // pull CS2 high (inactive)
    } 
    else
    {
        LATBbits.LATB5 = 1;                         // pull CS3 high (inactive)
    }  
}


/* ===========================================================================
 *
 * 
 * 
 *                      ALL I2C FUNCTIONS BELOW THIS
 * 
 * 
 * 
 * ========================================================================== */

int I2C_Read_Reg(char dev_addr, char reg_addr, char *value)
{
    char wr_dev_addr = dev_addr << 1;
    char rd_dev_addr = (dev_addr << 1) | 0x01;
    
    // Send I2C start condition
	I2C1CONbits.SEN = 1;	
	while(I2C1CONbits.SEN == 1);
    // Send I2C device address on the bus for read operation
    I2C1TRN = rd_dev_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;	
	}
    // Send I2C register address on the bus 
	I2C1TRN = reg_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{	
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;
	}
//    // Send I2C restart condition
//    I2C1CONbits.RSEN = 1;
//    while(I2C1CONbits.RSEN == 1);
//    // Enable I2C clock for read operation
//	I2C1CONbits.RCEN = 1;
//    while(!I2C1STATbits.RBF);
    // Retrieve value from I2C register
	*value = I2C1RCV;	
	// Send I2C stop condition
	I2C1CONbits.PEN = 1;
	while(I2C1CONbits.PEN);
	return I2C_OK;
}

void I2C_Init(void)
{
    I2C1CONbits.I2CEN = 0;      // disable I2C module	
	I2C1CONbits.I2CSIDL = 0;    // continue in idle mode
	I2C1CONbits.IPMIEN = 0;     // IMPI disabled
	I2C1CONbits.A10M = 0;       // 7-bit slave address
	I2C1CONbits.DISSLW = 1;     // slew rate control enabled
	I2C1CONbits.SMEN = 0;       // SMbus input thresholds disabled
    I2C1BRG = setBaudRate(1);   // set 100 kHz Baud Rate    
    //__delay_ms(1);
    I2C1CONbits.I2CEN = 1;      // enable I2C module
}

int setBaudRate(int codec)
{
    if (codec)
    {
        return (FCY/I2C_BAUDRATE_CODEC - FCY/1111111) - 1;
    }
    else
    {
        return (FCY/I2C_BAUDRATE - FCY/1111111) - 1;
    }
}

int I2C_WriteReg(char dev_addr, char reg_addr, char value)
{
    char wr_dev_addr = dev_addr << 1;
    // Send I2C start condition
	I2C1CONbits.SEN = 1;			
	while(I2C1CONbits.SEN == 1);
	// Send I2C device address on the bus for write operation
	I2C1TRN = wr_dev_addr;			
	while(I2C1STATbits.TRSTAT);			
	if (I2C1STATbits.ACKSTAT)				
	{								
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);			
		return I2C_ERROR;					
	}
    // Send register address on the bus
	I2C1TRN = reg_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;
	}
	// Send register value on the bus    
	I2C1TRN = value;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;
	}
	/// Send I2C stop condition
	I2C1CONbits.PEN = 1;
	while(I2C1CONbits.PEN);
	return I2C_OK;
}

int I2C_ReadReg(char dev_addr, char reg_addr, char *value)
{
    char wr_dev_addr = dev_addr << 1;
    char rd_dev_addr = (dev_addr << 1) | 0x01;
    
    // Send I2C start condition
	I2C1CONbits.SEN = 1;	
	while(I2C1CONbits.SEN == 1);
	// Send I2C device address on the bus for write operation
	I2C1TRN = wr_dev_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;
	}
    // Send I2C register address on the bus 
	I2C1TRN = reg_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{	
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;
	}
    // Send I2C restart condition
    I2C1CONbits.RSEN = 1;
    while(I2C1CONbits.RSEN == 1);	
    // Send I2C device address on the bus for read operation
    I2C1TRN = rd_dev_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;	
	}
    // Enable I2C clock for read operation
	I2C1CONbits.RCEN = 1;
    while(!I2C1STATbits.RBF);
    // Retrieve value from I2C register
	*value = I2C1RCV;	
	// Send I2C stop condition
	I2C1CONbits.PEN = 1;
	while(I2C1CONbits.PEN);
	return I2C_OK;
}

int I2C_ReadPIC(char data_addr, char *value)
{
    data_addr = (data_addr | 0x01);
    
    // Send I2C start condition
	I2C1CONbits.SEN = 1;	
	while(I2C1CONbits.SEN == 1);
	// Send I2C device address on the bus for write operation
	I2C1TRN = data_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;
	}
    // Enable I2C clock for read operation
	I2C1CONbits.RCEN = 1;
    while(!I2C1STATbits.RBF);
    // Retrieve value from I2C register
	*value = I2C1RCV;	
	// Send I2C stop condition
	I2C1CONbits.PEN = 1;
	while(I2C1CONbits.PEN);
	return I2C_OK;
}


