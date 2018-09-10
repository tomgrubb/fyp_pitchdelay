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
 *     08/12/2017  - Setup and test SPI for SRAM.
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
 * 
 * ------------------------------------------------------------------------
 * 
 *     Created November 3rd, 2017, 5:35 PM
 * 
 *     Last edit August 12th, 2018, 11:09 PM
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
#include "dma.h"
#include "p33Fxxxx.h"
#include "dsp.h"
#include "control_tables.h"  
#include "global.h"
#include "testSignal.h"

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

// ==================================================================================


/* ===========================================================================
 * 
 *                      INTERRUPT SERVICE ROUTINES
 *
 * ========================================================================== */

// TIMER 1 ISR - toggles LED on RA0
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;           // clear TIMER1 flag    
    //LATAbits.LATA0 ^= 1;        // toggle RA0
}

// DMA0 ISR - indicate that TX buffer is empty, set flag
void __attribute__((__interrupt__,no_auto_psv)) _DMA0Interrupt(void)
{
    IFS0bits.DMA0IF = 0;        // clear DMA0 IRQ flag
    //LATAbits.LATA0 ^= 1;        // toggle RA0
    flagTX = 1;                 // transfer buffer is empty 
}

// DMA1 ISR - indicate that RX buffer is empty, set flag
void __attribute__((__interrupt__,no_auto_psv)) _DMA1Interrupt(void)
{
    IFS0bits.DMA1IF = 0;        // clear DMA1 IRQ flag
    LATAbits.LATA0 ^= 1;
    flagRX = 1;                 // receive buffer is full
}

// ===================== [ IRQ FUNCTIONS END] ==============================

// increment each delay taps pointers, wrap around if necessary
void delayPtrIncrement(void)
{
    unsigned long testMax = 65536;
    
    // Delay Line 'A'
    writePtrA += (2*FRAME);
    
    // approaching the end of the buffer
    if (RAM_WritePtrA == maxMemA)
    {
        if (writePtrA >= timeA)
        {
            writePtrA = 0;
            RAM_WritePtrA = 0;
        }
    }
    else
    {
        if (writePtrA >= maxPtr)
        {
            writePtrA = 0;
            RAM_WritePtrA++;
        }
    }
    
    readPtrA += (2*FRAME);
    
    // approaching the end of the buffer
    if (RAM_ReadPtrA == maxMemA)
    {
        if (readPtrA >= timeA)
        {
            readPtrA = 0;
            RAM_ReadPtrA = 0;
        }
    }
    else
    {
        if (readPtrA >= maxPtr)
        {
            readPtrA = 0;
            RAM_ReadPtrA++;
        }
    }
    
    // Delay Line 'B'
    writePtrB += (2*FRAME);
    
    // approaching the end of the buffer
    if (RAM_WritePtrB == maxMemA)
    {
        if (writePtrB >= timeA)
        {
            writePtrB = 0;
            RAM_WritePtrB = 0;
        }
    }
    else
    {
        if (writePtrB >= maxPtr)
        {
            writePtrB = 0;
            RAM_WritePtrB++;
        }
    }
    
    readPtrB += (2*FRAME);
    
    // approaching the end of the buffer
    if (RAM_ReadPtrB == maxMemA)
    {
        if (readPtrB >= timeA)
        {
            readPtrB = 0;
            RAM_ReadPtrB = 0;
        }
    }
    else
    {
        if (readPtrB >= maxPtr)
        {
            readPtrB = 0;
            RAM_ReadPtrB++;
        }
    }
}

//void crossFade(int numElems, fractional* dstV, fractional* srcV1, fractional* srcV2)
//{
//    // scale vector 1 by crossfade gain structure
//    VectorMultiply(numElems, srcV1, srcV1, &crossFadeGainA);
//    // scale vector 2 by crossfade gain structure
//    VectorMultiply(numElems, srcV1, srcV2, &crossFadeGainB);
//    // sum scaled vectors
//    VectorAdd(numElems, dstV, srcV1, dstV);
//}

void FFTroutine(int *input, int *output)
{
    int i; 
    int	peakFrequencyBin = 0;				/* Declare post-FFT variables to compute the */
    unsigned long peakFrequency = 0;			/* frequency of the largest spectral component */
 
	fractional *p_real = &fractSignal[0].real ;
	fractcomplex *p_cmpx = &fractSignal[0] ;


	TwidFactorInit (LOG2_BLOCK_LENGTH, &twiddleFactorsFFT[0], 0);	/* We need to do this only once at start-up */

	for ( i = 0; i < FFT_BLOCK_LENGTH; i++ )/* The FFT function requires input data */
	{					/* to be in the fractional fixed-point range [-0.5, +0.5]*/
		*p_real = *p_real >>1 ;		/* So, we shift all data samples by 1 bit to the right. */
		*p_real++;			/* Should you desire to optimize this process, perform */
	}					/* data scaling when first obtaining the time samples */
						/* Or within the BitReverseComplex function source code */

	p_real = &fractSignal[(FFT_BLOCK_LENGTH/2)-1].real ;	/* Set up pointers to convert real array */
	p_cmpx = &fractSignal[FFT_BLOCK_LENGTH-1] ; /* to a complex array. The input array initially has all */
						/* the real input samples followed by a series of zeros */


	for ( i = FFT_BLOCK_LENGTH; i > 0; i-- ) /* Convert the Real input sample array */
	{					/* to a Complex input sample array  */
		(*p_cmpx).real = (*p_real--);	/* We will simpy zero out the imaginary  */
		(*p_cmpx--).imag = 0x0000;	/* part of each data sample */
	}

	FFTComplexIP (LOG2_BLOCK_LENGTH, &fractSignal[0], &twiddleFactorsFFT[0], COEFFS_IN_DATA);

	/* Store output samples in bit-reversed order of their addresses */
	BitReverseComplex (LOG2_BLOCK_LENGTH, &fractSignal[0]);

	/* Compute the square magnitude of the complex FFT output array so we have a Real output vetor */
	SquareMagnitudeCplx(FFT_BLOCK_LENGTH, &fractSignal[0], &fractSignal[0].real);

	/* Find the frequency Bin ( = index into the SigCmpx[] array) that has the largest energy*/
	/* i.e., the largest spectral component */
	VectorMax(FFT_BLOCK_LENGTH/2, &fractSignal[0].real, &peakFrequencyBin);

	/* Compute the frequency (in Hz) of the largest spectral component */
	peakFrequency = peakFrequencyBin*(10000/FFT_BLOCK_LENGTH);
}

void processRxData(void)
{   
    if (count < 1000)
    {
        count++;
    }
    else
    {
        if (buffer)
        {
            //loopBack(rxBufferA, txBufferA);
            //FFTroutine(fractSignal, bufferA);
            //delayEngine(rxBufferA, bufferA, RAM_ReadPtrA, RAM_WritePtrA, readPtrA, writePtrA, fbkA);
            //delayEngine(rxBufferA, bufferB, RAM_ReadPtrB, RAM_WritePtrB, readPtrB, writePtrB, fbkB);
            holdEngine(rxBufferA, bufferA, holdBuffer, RAM_ReadPtrA, RAM_WritePtrA, readPtrA, writePtrA, fbkA);
            mixer(bufferA, bufferB, bufferC, lvlA, 0x0, 0x0, txBufferA);
        }
        else
        {
            //loopBack(rxBufferB, txBufferB);
            //FFTroutine(fractSignal, bufferA);
            //delayEngine(rxBufferB, bufferA, RAM_ReadPtrA, RAM_WritePtrA, readPtrA, writePtrA, fbkA);
            //delayEngine(rxBufferB, bufferB, RAM_ReadPtrB, RAM_WritePtrB, readPtrB, writePtrB, fbkB);
            holdEngine(rxBufferB, bufferA, holdBuffer, RAM_ReadPtrA, RAM_WritePtrA, readPtrA, writePtrA, fbkA);
            mixer(bufferA, bufferB, bufferC, lvlA, 0x0, 0x0, txBufferB);
        }
        delayPtrIncrement();
    }
}

void FFTinit(void)
{
    TwidFactorInit(LOG2_BLOCK_LENGTH, &twiddleFactorsFFT[0], 0x0); 
    TwidFactorInit(LOG2_BLOCK_LENGTH, &twiddleFactorsIFFT[0], 0x1);
    HanningInit(FFT_BLOCK_LENGTH, &window);
}

void system_setup(void)
{
    clock_init();               // setup PLL clock for 40 MIPS
    interrupt_purge();          // ensure IRQ config not awry
    FFTinit();
    system_init();              // define IO states and startup states
    
//    interrupts_init();          // global interrupt setup
//    
//    I2C_Init();
//    
//    codec_setup();              // configure CODEC via I2C
//    DMA_init();                 // configure DMA0 and DMA1 for ping-pong RX/TX
//    I2S_init();                 // configure DCI for I2S with DMA
//    
//    SPI_init();                 // configure SPI module for SRAM
//    purge_RAM();                // ensure all RAM is empty
//    I2Sbuffer_init();           // fill RX and TX buffers with null values
//    
//    I2S_start();                // start up I2S and for initial DMA transfer
}

void fetchData(void)
{
    I2C_ReadPIC(0xA0, &testValue);   // retrieve data
    testValue = testValue>>1;
    fbkA = fbkValues[testValue];

    I2C_ReadPIC(0xA4, &testValue);   // retrieve data
    testValue = testValue>>1;
    lvlA = levelValues[testValue];

    I2C_ReadPIC(0xA8, &testValue);   // retrieve data
    testValue = testValue>>1;
    timeA = timeValues[testValue];
    
    if (timeA >= maxPtr)
    {
        timeA -= maxPtr;
        maxMemA = 1;
        if (timeA >= maxPtr)
        {
            timeA -= maxPtr;
            maxMemA = 2;
        }
    }
    else (maxMemA = 0);

    if (timeA < 0)
    {
        timeA = 0;
    }
}

// =========================== [ MAIN LOOP] ===================================
int main(void)
{
    //BartlettInit(FRAME, &cFadeBartlett[0]);
    //system_setup();
    
    
    LATAbits.LATA1 = 0;
    LATAbits.LATA0 = 0;
    flagRX = 0;
    flagTX = 0;

        int i; 
        int	peakFrequencyBin = 0;				/* Declare post-FFT variables to compute the */
        unsigned long peakFrequency = 0;			/* frequency of the largest spectral component */

        fractional *p_real = &fractSignal[0].real;
        fractcomplex *p_cmpx = &fractSignal[0];
        
        TwidFactorInit (LOG2_BLOCK_LENGTH, &twiddleFactorsFFT[0], 0);	/* We need to do this only once at start-up */
        TwidFactorInit (LOG2_BLOCK_LENGTH, &twiddleFactorsIFFT[0], 0x1);	/* We need to do this only once at start-up */

        for ( i = 0; i < FFT_BLOCK_LENGTH; i++ )/* The FFT function requires input data */
        {					/* to be in the fractional fixed-point range [-0.5, +0.5]*/
            *p_real = *p_real >>1 ;		/* So, we shift all data samples by 1 bit to the right. */
            *p_real++;			/* Should you desire to optimize this process, perform */
        }					/* data scaling when first obtaining the time samples */
                            /* Or within the BitReverseComplex function source code */

        p_real = &fractSignal[(FFT_BLOCK_LENGTH/2)-1].real ;	/* Set up pointers to convert real array */
        p_cmpx = &fractSignal[FFT_BLOCK_LENGTH-1] ; /* to a complex array. The input array initially has all */
                            /* the real input samples followed by a series of zeros */


        for ( i = FFT_BLOCK_LENGTH; i > 0; i-- ) /* Convert the Real input sample array */
        {					/* to a Complex input sample array  */
            (*p_cmpx).real = (*p_real--);	/* We will simpy zero out the imaginary  */
            (*p_cmpx--).imag = 0x0000;	/* part of each data sample */
        }

        FFTComplexIP (LOG2_BLOCK_LENGTH, &fractSignal[0], &twiddleFactorsFFT[0], COEFFS_IN_DATA);
        BitReverseComplex (LOG2_BLOCK_LENGTH, &fractSignal[0]);
        IFFTComplexIP (LOG2_BLOCK_LENGTH, &fractSignal[0], &twiddleFactorsIFFT[0], COEFFS_IN_DATA);
        FFTComplexIP (LOG2_BLOCK_LENGTH, &fractSignal[0], &twiddleFactorsFFT[0], COEFFS_IN_DATA);
        /* Store output samples in bit-reversed order of their addresses */
        BitReverseComplex (LOG2_BLOCK_LENGTH, &fractSignal[0]);

        /* Compute the square magnitude of the complex FFT output array so we have a Real output vetor */
        SquareMagnitudeCplx(FFT_BLOCK_LENGTH, &fractSignal[0], &fractSignal[0].real);

        /* Find the frequency Bin ( = index into the SigCmpx[] array) that has the largest energy*/
        /* i.e., the largest spectral component */
        VectorMax(FFT_BLOCK_LENGTH/2, &fractSignal[0].real, &peakFrequencyBin);

        /* Compute the frequency (in Hz) of the largest spectral component */
        peakFrequency = peakFrequencyBin*(10000/FFT_BLOCK_LENGTH);
    
    while(1)
    {
       if (flagRX & flagTX)
       // if (1)
       {              
            LATAbits.LATA1 = 1;
            processRxData();
            LATAbits.LATA1 = 0;
            
            // reset flags and swap targeted buffer
            buffer ^= 1;
            flagRX = 0;
            flagTX = 0;
       }
       //fetchData();
    }
}
