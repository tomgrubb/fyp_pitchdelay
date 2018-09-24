/* ========================================================================
 *                        PITCH SHIFTER / DELAY
 * 
 *                           SIGNAL PROCESSOR
 * 
 *                          FINAL YEAR PROJECT
 *                          MONASH UNIVERSITY
 *                                2018
 * 
 *      File:   main.c
 *      Author: Thomas M. Grubb
 * 
 *                      (c) Sieger Audio 2017
 *
 * ------------------------- [CHANGE LOG] ---------------------------------
 * 
 *     03/11/2017  - Setup and test UART
 *     14/11/2017  - UART test OK
 *     21/11/2017  - I2S and DMA setup (adapted from MICROCHIP sample)
 *     28/11/2017  - Configure and test for PRI w/ PLL
 *                 - Setup I2C operation
 *     08/12/2017  - Setup and test SPI for SRAM
 *                 - SPI test OK, need to reconfigure for 8-bit TX/RX
 *                 - SPI configured for 8-bit TX/RX, test OK
 *     11/12/2017  - Setup and test DCI for I2S, clock signals test OK  
 *     12/12/2017  - I2S test OK, need to configure for 16 bit format
 *     19/12/2017  - CODEC setup for dithered 16-bit I2S operation  
 *                 - Setup DCI interrupts to loop back audio, test OK
 *                 - Audio is noisy, testing suggests grounding issues
 *     02/01/2018  - DAC filter fixed, clicking noise eliminated
 *                 - Setup and test short delay (~1ms) using dsPIC RAM 
 *                 - Setup and test comb filter 
 *     03/01/2018  - Begin reconfiguration for DCI as Master, setup DMA for
 *                   ping-pong RX and TX to enable more time for processing   
 *     05/01/2018  - Ping-pong between two buffers (handled by DMA0 and DMA1)
 *                   working. Constant, pitched hum has appeared. Appears to be
 *                   due to grounding / current spikes. Changing value of 'FRAME'
 *                   increases or decreases this pitch.
 * 	   12/04/2018  - Properly installed and linked DSP library.
 *   			   - Changed processes in delay engine to DSP functions
 *     26/04/2018  - Initial configuration of FIR filter to run interpolation
 *     27/04/2018  - FIR filter configured, tested interpolation and decimation.
 *     03/07/2018  - Configure and test collection of data from controller via I2C   
 *     12/08/2018  - Organized DSP processing functions into engines.c
 *                 - Fixed framing I2S framing issue that was causing 2x interrupts
 *                 - Parameterized DSP engine variables for stability/flexibility
 *                 - Coded and tested mixer function (for up to three inputs)
 *     24/09/2018  - Added adaptive SRAM read so enable reading across memory boundaries
 *                 - Single pole low pass implemented for changing delay times
 *                 - Delay time resolution increased to 12 bits (1024 values)
 *                 - Added starup check to ensure controller and processor are synced                        
 * 
 * ------------------------------------------------------------------------
 * 
 *     Created November 3rd, 2017, 5:35 PM
 * 
 *     Last edit September 24th, 2018, 11:26 AM
 * 
 * ======================================================================== */

// DEVICE CONFIG
// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRC              // External Primary Oscillator
#pragma config IESO = OFF               // Internal External Switch Over Mode OFF

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (HS Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is disabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// INCLUDED HEADERS
#include <xc.h>
#include "MACROS.h"
#include "main.h"
#include <stdint.h>
#include "math.h"
#include <string.h>
#include "libpic30.h"
//#include "dma.h"
#include "p33Fxxxx.h"
#include "dsp.h"
#include "control_tables.h"  
#include "global.h"
//#include "testSignal.h"

#define BL 11                           // number of coefficients for FIR filter
#define LOG2_BLOCK_LENGTH  		8            
#define FFT_BLOCK_LENGTH  		256          
#define IFFT_BLOCK_LENGTH  		256   

// transmit and receive buffers to ping-pong: A <--> B
int txBufferA[FRAME] __attribute__((space(dma)));
int txBufferB[FRAME] __attribute__((space(dma)));
int rxBufferA[FRAME] __attribute__((space(dma)));
int rxBufferB[FRAME] __attribute__((space(dma)));

int bufferA[FRAME];
int bufferB[FRAME];
int bufferC[FRAME];
int filteredInput[FRAME];

int timeValueBuffer[10];

int R = 2;
FIRStruct FIRfilter;
FIRStruct FIRlowpass;

int prevBuff[FRAME];
int tempCrossBuff[FRAME];
int tempPrevBuff[FRAME];

// =======================  FFT variables and arrays ================================

// FFT input window
//fractcomplex fftWkspace[FFT_BLOCK_LENGTH] __attribute__((space(ymemory), far, aligned(FFT_BLOCK_LENGTH*2*2))); 

//fractional cFadeBartlett[FRAME];

// windowing function
fractional window[FFT_BLOCK_LENGTH]  __attribute__((space(ymemory), aligned(FFT_BLOCK_LENGTH*2))); 

// Twiddle Factor array for FFT
fractcomplex twiddleFactorsFFT[FFT_BLOCK_LENGTH/2] __attribute__((space(xmemory), aligned(FFT_BLOCK_LENGTH*2))); 

// Twiddle Factor array for IFFT
fractcomplex twiddleFactorsIFFT[IFFT_BLOCK_LENGTH/2] __attribute__((space(xmemory), aligned(FFT_BLOCK_LENGTH*2*2))); 

// ===========================================================================


/* ===========================================================================
 * 
 *                      INTERRUPT SERVICE ROUTINES
 *
 * ========================================================================== */

// DMA0 ISR - indicate that TX buffer is empty, set flag
void __attribute__((__interrupt__,no_auto_psv)) _DMA0Interrupt(void)
{
    IFS0bits.DMA0IF = 0;        // clear DMA0 IRQ flag
    flagTX = 1;                 // transfer buffer is empty 
}

// DMA1 ISR - indicate that RX buffer is empty, set flag
void __attribute__((__interrupt__,no_auto_psv)) _DMA1Interrupt(void)
{
    IFS0bits.DMA1IF = 0;        // clear DMA1 IRQ flag
    flagRX = 1;                 // receive buffer is full
}

// ===================== [ IRQ FUNCTIONS END] ==============================

// increment each delay taps pointers, wrap around if necessary
void delayPtrIncrement(void)
{   
    // Delay Line 'A'
    writePtrA += BLOCK;     // increment write pointer by one block
    
    if (writePtrA > maxPtr)    // pointer is beyond memory boundary
    {
        writePtrA = 0;
        RAM_WritePtrA++;
        if (RAM_WritePtrA > maxMem[0])
        {
            RAM_WritePtrA = memStart[0];
        }
    }
}

void setDelayTap(long int delayTime)
{
    long int diff = 0;
    int rem = 0;
    int test = 0;
    double a = 0.9;
    double b = 1 - a;

    //timeA = a*nextTimeA + b*TimeA;    // implement low pass to move towards new delay time

    diff = timeA - nextTimeA;
    
    modCount++;
    if (modCount >= 0)
    {
        modCount = 0;
        mod += 2*(dir);
        if ((mod > 50)||(mod < -50))
        {
            dir = dir*(-1);
        }
    }

    diff = diff>>4;
    if ((-3 < diff) && (diff < 3))
    {
        timeA = nextTimeA;
    }
    else
    {
        timeA -= diff;
    }
    
    //timeA += mod;

    readPtrA = writePtrA - timeA;    
    RAM_ReadPtrA = RAM_WritePtrA;
    if (readPtrA < 0)
    {
        readPtrA += boundary;
        RAM_ReadPtrA--;
        if (RAM_ReadPtrA < memStart[0])
        {
            RAM_ReadPtrA = maxMem[0];
        }
    }
    
    rem = readPtrA % 2;
    test = rem;
    if (rem > 0)
    {
        readPtrA -= 1;
    }
    
}

void processRxData(void)
{   
    int *rxPtr;
    int *txPtr;
    
    if (count < 500)
    {
        count++;
    }
    else
    {
        if (buffer)
        {
            rxPtr = rxBufferA;
            txPtr = txBufferA;
        }
        else
        {
            rxPtr = rxBufferB;
            txPtr = txBufferB;
        }
        
        // DO THE PROCESSING
        blockDC(rxPtr, filteredInput, &xz1, &yz1);
        //loopBack(filteredInput, txPtr);
        //FFTroutine(fractSignal, bufferA);
        delayEngine(filteredInput, bufferA, RAM_ReadPtrA, RAM_WritePtrA, readPtrA, writePtrA, fbkA, 0);//, cFadePtrA, RAM_FadePtrA, fade);
        //delayEngine(rxBufferA, bufferB, RAM_ReadPtrB, RAM_WritePtrB, readPtrB, writePtrB, fbkB, 0);
        mixer(bufferA, bufferB, bufferC, lvlA, 0x0, 0x0, txPtr);
        
        delayPtrIncrement();
    }
}

void FFTinit(void)
{
    TwidFactorInit(LOG2_BLOCK_LENGTH, &twiddleFactorsFFT[0], 0x0); 
    TwidFactorInit(LOG2_BLOCK_LENGTH, &twiddleFactorsIFFT[0], 0x1);
    HanningInit(FFT_BLOCK_LENGTH, &window);
}

void initControls(void)
{
    // variable to concatenate data from multiple transfers
    int byteLo = 0;
    int byteHi = 0;
    
    // value of control voltage
    int potVal = 0;
    
    // read Feedback A control voltage, map to value
    I2C_ReadPIC(0xA0, &testValue);
    fbkA = fbkValues[testValue];   
    
    // read Level A control voltage, map to value
    I2C_ReadPIC(0xA4, &testValue);
    lvlA = levelValues[testValue];

    // read Time A control voltag
    I2C_ReadPIC(0xA8, &testValue);
    byteLo = testValue;
    I2C_ReadPIC(0xA8, &testValue);
    byteHi = testValue;
    // concatenate to form 12 bit value, map to value 
    potVal = ((byteHi << 8) | (byteLo & 0xFF));
    timeA = timeValues[potVal];
    
    readPtrA = writePtrA - timeA;
    RAM_ReadPtrA = RAM_WritePtrA;
    if (readPtrA < 0)
    {
        readPtrA += boundary;
        RAM_ReadPtrA--;
        if (RAM_ReadPtrA < memStart[0])
        {
            RAM_ReadPtrA = maxMem[0];
        }
    }
    
}

// read input parameters from control system
void fetchData(void)
{   
    // variable to concatenate data from multiple transfers
    int byteLo = 0;
    int byteHi = 0;
    
    // value of control voltage
    int potVal = 0;
    
    // read Feedback A control voltage, map to value
    I2C_ReadPIC(0xA0, &testValue);
    fbkA = fbkValues[testValue];   
    
    // read Level A control voltage, map to value
    I2C_ReadPIC(0xA4, &testValue);
    lvlA = levelValues[testValue];

    // read Time A control voltag
    I2C_ReadPIC(0xA8, &testValue);
    byteLo = testValue;
    I2C_ReadPIC(0xA8, &testValue);
    byteHi = testValue;
    // concatenate to form 12 bit value, map to value 
    potVal = ((byteHi << 8) | (byteLo & 0xFF));
    //potVal = potVal;
    nextTimeA = timeValues[potVal];
    
    setDelayTap(0);
}

void confirmStartup(void)
{
    int startupComplete = 0;
    
    while (!startupComplete)
    {
        I2C_ReadPIC(0xAC, &testValue);
        startupComplete = testValue;
    }
}

void systemSetup(void)
{
    clock_init();               // setup PLL clock for 40 MIPS
    interrupt_purge();          // ensure IRQ config not awry
    //FFTinit();
    //FIRDelayInit(&FIRdcblock);
    
    system_init();              // define IO states and startup states
    
    interrupts_init();          // global interrupt setup
    
    I2C_Init();                 // setup I2C link to controller
    fetchData();                // collect initial parameters
    codec_setup();              // configure CODEC via I2C
    DMA_init();                 // configure DMA0 and DMA1 for ping-pong RX/TX
    I2S_init();                 // configure DCI for I2S with DMA
    
    initControls();             // set params based on control startup values
    
    SPI_init();                 // configure SPI module for SRAM
    purge_RAM();                // ensure all RAM is empty
    I2Sbuffer_init();           // fill RX and TX buffers with null values
    
    confirmStartup();           // ensure that controller is ready for operation
    
    I2S_start();                // start up I2S and for initial DMA transfer
}

// =========================== [ MAIN LOOP] ===================================
int main(void)
{    
    systemSetup();

    // just in case flags got set during setup
    flagRX = 0;
    flagTX = 0;
    
    while(1)
    {
       if (flagRX & flagTX)
       {    
            processRxData(); // all processing of audio happens now
            
            // reset flags and swap targeted buffer
            buffer ^= 1;
            flagRX = 0;
            flagTX = 0;
           
            fetchData(); // probe UI for new parameter values
       }
    }
}
