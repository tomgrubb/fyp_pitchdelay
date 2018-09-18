#include "xc.h"
#include "MACROS.h"
#include "crossfade.h"
#include "dsp.h"

void SPI_SRAM_BlockWrite(int CS, unsigned long int reg_add, int data[]);
void SPI_SRAM_BlockRead(int CS, unsigned long int reg_add, int value[], int elements);

int crossScaleA[8] = { 0x8000, 0x7000, 0x6000, 0x5000, 0x4000, 0x3000, 0x2000, 0x1000 };
int crossScaleB[8] = { 0x1000, 0x2000, 0x3000, 0x4000, 0x5000, 0x6000, 0x7000, 0x8000 };

// 'DELAY' MODE
void delayEngine(int *inBuff, int *outBuff, int ptrRdRAM, int ptrWrRAM, long int readPtr, long int writePtr, int fbk, char line, unsigned long cFadePtr, int cFadeRAM, int fade)
{
    int tempBuff[FRAME];
    int cFadeBuff[FRAME];
    int invGain[FRAME];
    int n;
    int posFade = 0;
    int negFade = 0;
    
    long int testPtr = 0;
   
    if (fade > 0)   // cross-fading between two delay times
    {
        testPtr = cFadePtr + BLOCK;
        if (testPtr <= boundary)
        {
            // read block is within RAM memory boundary
            SPI_SRAM_BlockRead(cFadeRAM, cFadePtr, cFadeBuff, FRAME);
        }

        else
        {
            // read block crosses RAM memory boundary
            SPI_SRAM_BoundaryRead(cFadeRAM, cFadePtr, cFadeBuff, line);
        }
    }
    
    testPtr = readPtr + BLOCK;
    
    // read audio from delay line into output buffer     
    if (testPtr <= boundary)
    {
        // read block is within RAM memory boundary
        SPI_SRAM_BlockRead(ptrRdRAM, readPtr, outBuff, FRAME);
    }
    else
    {
        // read block crosses RAM memory boundary
        SPI_SRAM_BoundaryRead(ptrRdRAM, readPtr, outBuff, line);
    }
    
    if (fade > 0)
    {
        n = fade - 1;
        VectorScale(FRAME, outBuff, outBuff, crossScaleA[n]);
        VectorScale(FRAME, &cFadeBuff, &cFadeBuff, crossScaleB[n]);
        
//        if (fade == 1)
//        {
//            VectorScale(FRAME, outBuff, outBuff, 0x7000);
//            VectorScale(FRAME, &cFadeBuff, &cFadeBuff, 0x1000);
////            for (n = 0; n < FRAME; n++)
////            {
////                invGain[n] = 0x8000 - cFadePosA[n];
////            }
////            VectorMultiply(FRAME, outBuff, outBuff, &invGain);
////            VectorMultiply(FRAME, &cFadeBuff, &cFadeBuff, &cFadePosA);
//        }
//        else if (fade == 2)
//        {
//            VectorScale(FRAME, outBuff, outBuff, 0x5000);
//            VectorScale(FRAME, &cFadeBuff, &cFadeBuff, 0x3000);
////            for (n = 0; n < FRAME; n++)
////            {
////                invGain[n] = 0x8000 - cFadePosB[n];
////            }
////            VectorMultiply(FRAME, outBuff, outBuff, &invGain);
////            VectorMultiply(FRAME, &cFadeBuff, &cFadeBuff, &cFadePosB);
//        }
//        else if (fade == 3)
//        { 
//            VectorScale(FRAME, outBuff, outBuff, 0x4000);
//            VectorScale(FRAME, &cFadeBuff, &cFadeBuff, 0x4000);
////            for (n = 0; n < FRAME; n++)
////            {
////                invGain[n] = 0x8000 - cFadePosC[n];
////            }
////            VectorMultiply(FRAME, outBuff, outBuff, &invGain);
////            VectorMultiply(FRAME, &cFadeBuff, &cFadeBuff, &cFadePosC);
//        }
//        else
//        {   
//            VectorScale(FRAME, outBuff, outBuff, 0x2000);
//            VectorScale(FRAME, &cFadeBuff, &cFadeBuff, 0x6000);
////            for (n = 0; n < FRAME; n++)
////            {
////                invGain[n] = 0x8000 - cFadePosD[n];
////            }
////            VectorMultiply(FRAME, outBuff, outBuff, &invGain);
////            VectorMultiply(FRAME, &cFadeBuff, &cFadeBuff, &cFadePosD);
//        }
        VectorAdd(FRAME, outBuff, outBuff, &cFadeBuff);
    }
    
    // scale output and store in feedback buffer
    VectorScale(FRAME, &tempBuff, outBuff, fbk);
    // sum feedback and input buffers
    VectorAdd(FRAME, &tempBuff, inBuff, &tempBuff);
    // write mixed audio to delay line
    SPI_SRAM_BlockWrite(ptrWrRAM, writePtr, tempBuff);
}

// 'HOLD' MODE
void holdEngine(int *inBuff, int *outBuff, int *prevGrain, int ptrRdRAM, int ptrWrRAM, long int readPtr, long int writePtr, int fbk)
{
    int tempBuff[FRAME];
    
    // read audio from delay line into temporary buffer
    SPI_SRAM_BlockRead(ptrRdRAM, readPtr, &tempBuff, FRAME);
    // multiple new grain by positive cross fade coefficients
    VectorScale(FRAME, &tempBuff, outBuff, &tempBuff);
    // scale output and store in feedback buffer
    VectorScale(FRAME, &tempBuff, outBuff, fbk);
    // sum feedback and input buffers
    VectorAdd(FRAME, &tempBuff, inBuff, &tempBuff);
    // write mixed audio to delay line
    SPI_SRAM_BlockWrite(ptrWrRAM, writePtr, &tempBuff);
}

void mixer(int *buffA, int *buffB, int *buffC, int lvlA, int lvlB, int lvlC, int *outBuff)
{
    VectorScale(FRAME, buffA, buffA, lvlA); // scale buffer A
    VectorScale(FRAME, buffB, buffB, lvlB); // scale buffer B
    VectorScale(FRAME, buffC, buffC, lvlC); // scale buffer C
    
    // sum all buffer to output
    VectorAdd(FRAME, outBuff, buffA, buffB);
    VectorAdd(FRAME, outBuff, outBuff, buffC);
}

// pass RX buffer to TX buffer
void loopBack(int *inPtr, int *outPtr)
{
    VectorCopy(FRAME, outPtr, inPtr);
}

void crossFade(int numElems, int *dstV, int *srcV1, int *srcV2, int *cFadeGainA, int *cFadeGainB)
{
    // scale vector 1 by crossfade gain structure
    VectorMultiply(numElems, srcV1, srcV1, cFadeGainA);
    // scale vector 2 by crossfade gain structure
    VectorMultiply(numElems, srcV1, srcV2, cFadeGainB);
    // sum scaled vectors
    VectorAdd(numElems, dstV, srcV1, dstV);
}