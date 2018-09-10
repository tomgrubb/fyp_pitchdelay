/*
 * File:   DCI.c
 * Author: Tom Grubb
 *
 * Created on November 20, 2017, 11:01 AM
 */


#include "xc.h"
#include "MACROS.h"
#include <stdint.h>
#include "libpic30.h"

extern int txBufferA[FRAME] __attribute__((space(dma)));
extern int txBufferB[FRAME] __attribute__((space(dma)));
extern int rxBufferA[FRAME] __attribute__((space(dma)));
extern int rxBufferB[FRAME] __attribute__((space(dma)));

// DCI Initialization in I2S mode
void I2S_init(void)
{
    /*
    // [ dcPIC IS MASTER]
    // DCI Control Register 1
    DCICON1 = 0;
    DCICON1bits.DCIEN = 0;      // disable DCI
    DCICON1bits.DCISIDL = 0;    // operation continues when CPU is idle
    DCICON1bits.DLOOP = 0;      // disable digital loop back
    DCICON1bits.CSCKD = 0;      // CSCK pin is output (dsPIC controlled)
    DCICON1bits.CSCKE = 0;      // sample rising edge, data changes on falling edge
    DCICON1bits.COFSD = 0;      // COFS pin is output (dsPIC controlled)
    DCICON1bits.UNFM = 0;       // transmit zeros upon underflow
    DCICON1bits.CSDOM = 0;      // CSDOM pin drives LOW during disable
    DCICON1bits.DJST = 0;       // TX/RX starts one cycle after frame sync
    DCICON1bits.COFSM = 1;      // I2S Frame Sync mode
         
    // DCI Control Register 2
    DCICON2bits.BLEN = 0x0;       // one data word buffered between interrupts
    DCICON2bits.COFSG = 0x1;      // data frame has one word
    DCICON2bits.WS = 0xF;         // data word size is 16 bits
    
    // DCI Control Register 3
    DCICON3 = BCGVAL;             // clock is driven by dsPIC
    
    TSCON = 0x0003;               // enable two TX slot
    RSCON = 0x0003;               // enable two RX slot
    
    */
    
    //                    [ CODEC IS MASTER ]
    // DCI Control Register 1
    //DCICON1 = 0;
    DCICON1bits.DCIEN = 0;      // disable DCI
    DCICON1bits.DCISIDL = 0;    // operation continues when CPU is idle
    DCICON1bits.DLOOP = 0;      // disable digital loop back
    DCICON1bits.CSCKD = 1;      // CSCK pin is input (CODEC controlled)
    DCICON1bits.CSCKE = 1;      // sample rising edge, data changes on falling edge
    DCICON1bits.COFSD = 1;      // COFS pin is input (CODEC controlled)
    DCICON1bits.UNFM = 0;       // transmit zeros upon underflow
    DCICON1bits.CSDOM = 0;      // CSDOM pin drives LOW during disable
    DCICON1bits.DJST = 0;       // TX/RX starts one cycle after frame sync
    DCICON1bits.COFSM = 1;      // I2S Frame Sync mode
     
    
//    // DCI Control Register 2
//    DCICON2bits.BLEN = 0x1;       // one data word buffered between interrupts
//    DCICON2bits.COFSG = 0x1;      // data frame has one word
//    DCICON2bits.WS = 0xF;         // data word size is 16 bits
    
    
    //  WORKING!!!
//    // DCI Control Register 2
//    DCICON2bits.BLEN = 0x0;       // one data word buffered between interrupts
//    DCICON2bits.COFSG = 0x0;      // data frame has one word
//    DCICON2bits.WS = 0xF;         // data word size is 16 bits
    
    // DCI Control Register 2
    DCICON2bits.BLEN = 0x0;       // one data word buffered between interrupts
    DCICON2bits.COFSG = 0x3;      // data frame has four words
    DCICON2bits.WS = 0xF;         // data word size is 16 bits
    
    // DCI Control Register 3
    DCICON3 = 0;                  // clock is driven by CODEC      
    
    TSCON = 0x00;
    RSCON = 0x00;
    
    TSCONbits.TSE2 = 1;
    RSCONbits.RSE2 = 1;
}

// initialize transfer buffers 
void I2Sbuffer_init(void)   
{   
    unsigned int i;   
  
    for(i = 0; i < FRAME; i++)    
    {   
        txBufferA[i]= 0x0;    
        txBufferB[i]= 0x0;
    }    
}

void I2S_start(void)   
{
    //TXBUF0 = 0xAAFF;
    //TXBUF1 = 0xFAFA;
    //temp = RXBUF0;
    //temp = RXBUF1;
    // Disable DCI Interrupt and Enable DCI module
//    IPC15bits.DCIIP = 6;          // Enable DCI interrupts  
//    IFS3bits.DCIIF = 0;           // DCI event interrupt flag 
//    IEC3bits.DCIIE = 0;           // enable DCI Event interrupts
    
    DMA0REQbits.FORCE = 1;
    while(DMA0REQbits.FORCE == 1);
    DMA0REQbits.FORCE = 1;
    while(DMA0REQbits.FORCE == 1);
    
    DCICON1bits.DCIEN = 1;        // enable DCI module
}  

// initialize DMA0 to handle I2S
void DMA_init(void)
{
    // DMA0: DMA RAM --> DCI (Transfer)
    DMA0CONbits.SIZE = 0;           // word transfer 
    DMA0CONbits.DIR = 1;            // from DMA RAM to DCI
    DMA0CONbits.AMODE = 0;          // register indirect with post-increment mode
    DMA0CONbits.MODE = 2;           // continuous ping-pong mode
    DMA0CONbits.HALF = 0;           // interrupt when all data is moved
    DMA0CONbits.NULLW = 0;          // normal operation
    DMA0REQbits.FORCE = 0;          // transfer is automatic
    DMA0REQbits.IRQSEL = 0x3C;      // IRQ is CODEC transfer complete
    
    DMA0STA = __builtin_dmaoffset(txBufferA);   // RAM A start address
    DMA0STB = __builtin_dmaoffset(txBufferB);   // RAM B start address
    DMA0PAD = (volatile unsigned int) &TXBUF0;  // peripheral address register
    DMA0CNT = FRAME-1;                          // transfer count
    
    IFS0bits.DMA0IF = 0;
    IPC1bits.DMA0IP = 6;
    IEC0bits.DMA0IE = 1;
    
    // DMA1: DCI --> DMA RAM  (Receive)
    DMA1CONbits.SIZE = 0;           // word transfer
    DMA1CONbits.DIR = 0;            // from DCI to DMA RAM
    DMA1CONbits.HALF = 0;           // interrupt when all data is moved
    DMA1CONbits.NULLW = 0;          // normal operation
    DMA1CONbits.AMODE = 0;          // register indirect with post-increment mode
    DMA1CONbits.MODE = 2;           // continuous ping-pong mode
    DMA1REQbits.FORCE = 0;          // transfer is automatic
    DMA1REQbits.IRQSEL = 0x3C;      // IRQ is CODEC transfer complete
    
    DMA1STA = __builtin_dmaoffset(rxBufferA);    // RAM A start address
    DMA1STB = __builtin_dmaoffset(rxBufferB);    // RAM B start address
    DMA1PAD = (volatile unsigned int) &RXBUF0;  // peripheral address register
    DMA1CNT = FRAME-1;                          // transfer count     
    
    IFS0bits.DMA1IF = 0;
    IPC3bits.DMA1IP = 7;
    IEC0bits.DMA1IE = 1;
    
    // enable DMA channels 0 and 2
    DMA0CONbits.CHEN = 1;               
    DMA1CONbits.CHEN = 1;  
} 