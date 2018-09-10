/*
 * File:   system.c
 * Author: tom
 *
 * Created on January 7, 2018, 3:32 PM
 */


#include <xc.h>
#include <p33FJ128GP802.h>
#include "MACROS.h"
#include "libpic30.h"
#include "p33Fxxxx.h"

void SPI_SRAM_Write(int CS, unsigned long int reg_add, int data);

// setup and switch clock
void clock_init(void)
{
    CLKDIVbits.PLLPRE = 2;              // PRI/4 = 5 MHz
    PLLFBDbits.PLLDIV = 30;             // x 32 = 160 MHz
    CLKDIVbits.PLLPOST = 0;             // divide by 2 = 80 MHz  
    __builtin_write_OSCCONH(0x03);      // next clock is HS PRI w/ PLL
    __builtin_write_OSCCONL(0x01);		// start clock switching
    while (OSCCONbits.COSC != 3);       // wait for current clock to be HS PRI w/ PLL
}

void system_init(void)
{
    AD1PCFGL = 0xFFFF;          // all pins DIGITAL
    ADPCFGbits.PCFG3 = 1;       // pin RB3 is DIGITAL 
    ADPCFGbits.PCFG4 = 1;
    //ADPCFGbits.PCFG12 = 1;      // pin RB12 is DIGITAL
            
    TRISAbits.TRISA0 = 0;       // define pin RA0 as OUTPUT
    TRISAbits.TRISA1 = 0;       // define pin RA1 as OUTPUT
    TRISAbits.TRISA4 = 0;       // define pin RA4 as OUTPUT
    TRISBbits.TRISB3 = 0;       // define pin RB3 as OUTPUT
    TRISBbits.TRISB4 = 0;       // define pin RB4 as OUTPUT    
    TRISBbits.TRISB5 = 0;       // define pin RB5 as OUTPUT
    TRISBbits.TRISB6 = 0;       // define pin RB6 as OUTPUT    
    TRISBbits.TRISB7 = 0;       // define pin RB7 as OUTPUT
    
    LATAbits.LATA0 = 1;         // pull RA0 pin HIGH
    LATAbits.LATA1 = 0;         // pull RA1 pin LOW
    LATAbits.LATA4 = 1;         // pull RA4 pin HIGH
    LATBbits.LATB3 = 1;         // pull RB3 pin HIGH
    LATBbits.LATB4 = 1;         // pull RB4 pub HIGH
    LATBbits.LATB5 = 1;         // pull RB5 pin HIGH
    LATBbits.LATB6 = 0;         // pull RB6 pin LOW
    LATBbits.LATB7 = 0;         // pull RB7 pin LOW

    // Assign peripheral pins (SPI)
    __builtin_write_OSCCONL(OSCCON & 0xDF);         // clear IOLOCK 
    RPOR0bits.RP0R = 0x8;                           // SCL is RP0 (PIN 4)
    RPINR20bits.SDI1R = 0x1;                        // SDI is RP1 (PIN 5)
    RPOR1bits.RP2R = 0x7;                           // SDO is RP2 (PIN 6)
    
//    // USE THESE ASSIGNMENTS FOR NEW PROTO!!
//    RPOR0bits.RP0R = 0x7;                           // SDO1 is RP0 (PIN 4)
//    RPOR2bits.RP5R = 0x8;                           // SCL1 is RP1 (PIN 5)
//    RPINR20bits.SDI1R = 0x2;                        // SDI1 is RP2 (PIN 6)
    
    // Assign peripheral pins (I2C / DCI)
    /*
    // dsPIC IS MASTER
    RPOR7bits.RP15R = 0xF;                          // COFS is RP15  (PIN 26)
    RPOR7bits.RP14R = 0xE;                          // CSCSK is RP14 (PIN 25)
    RPINR24bits.CSDIR = 0xD;                        // CSDI is RP13  (PIN 24)
    RPOR6bits.RP12R = 0xD;                          // CSDO is RP12  (PIN 23)
    */
    // CODEC IS MASTER 
    RPINR25bits.COFSR = 0xF;                        // COFS is RP15  (PIN 26)
    RPINR24bits.CSCKR = 0xE;                        // CSCSK is RP14 (PIN 25)
    RPINR24bits.CSDIR = 0xD;                        // CSDI is RP13  (PIN 24)
    RPOR6bits.RP12R = 0xD;                          // CSDO is RP12  (PIN 23)
    __builtin_write_OSCCONL(OSCCON | 0x40);         // set IOLOCK
}

void interrupt_purge(void)
{
    // Just in case something funky happened during power up....
    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
    IFS3 = 0;
    IFS4 = 0;
    IEC0 = 0;
    IEC1 = 0;
    IEC2 = 0;
    IEC3 = 0;
    IEC4 = 0;
    // Now we're ready to go
}

void TIMER1_init(void)
{
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0x7;
    T1CONbits.TGATE = 0;
    T1CONbits.TSIDL = 0;

    IPC0bits.T1IP = 5;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;

    T1CONbits.TON = 1;
}

void interrupts_init(void)
{
    INTCON1bits.NSTDIS = 0;     // disable nested interrupts
    IFS3bits.DCIIF = 0;         // clear DCI Event Interrupt flag
    IFS3bits.DCIEIF = 0;        // clear DCI Error Interrupt flag
    SRbits.IPL = 0;             // CPU priority
}

void codec_setup(void)
{
    __delay_ms(10);                             // wait for Vdd to settle
    LATBbits.LATB6 = 1;                         // pull !RST pin HIGH    
    __delay_ms(6);
    I2C_WriteReg(CODEC_ADD, 0x07, 0x03);        // enter Control Port Mode
    I2C_WriteReg(CODEC_ADD, 0x01, 0b01001001);  // Mode 1 Reg 
    I2C_WriteReg(CODEC_ADD, 0x02, 0b01100000);  // DAC Control Reg
    I2C_WriteReg(CODEC_ADD, 0x06, 0b00110100);  // ADC Control Reg
    I2C_WriteReg(CODEC_ADD, 0x07, 0x02);        // exit Low Power State 
    __delay_ms(10);                             // wait for reset to complete
    I2C1CONbits.I2CEN = 0;                      // disable I2C module
    I2C1BRG = setBaudRate(0);                   // set 400 kHz Baud Rate 
    I2C1CONbits.I2CEN = 1;                      // enable I2C module
}

// write zeros to all RAM block over all addresses
void purge_RAM(void)
{
    int i = 0;
    int j = 0;
    int blank = 0x0;
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < maxPtr; j++)
        {
            SPI_SRAM_Write(i, j, blank);
        }
    }
}

