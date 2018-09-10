#include "xc.h"
#include "MACROS.h"
#include "hold.h"
#include "dsp.h"

void SPI_SRAM_BlockWrite(int CS, unsigned long int reg_add, int data[]);
void SPI_SRAM_BlockRead(int CS, unsigned long int reg_add, int value[]);

// 'DELAY' MODE
void delayEngine(int *inBuff, int *outBuff, int ptrRdRAM, int ptrWrRAM, long int readPtr, long int writePtr, int fbk)
{
    int tempBuff[FRAME];
    
    // read audio from delay line into output buffer
    SPI_SRAM_BlockRead(ptrRdRAM, readPtr, outBuff);
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
    SPI_SRAM_BlockRead(ptrRdRAM, readPtr, &tempBuff);
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