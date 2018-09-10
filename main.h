#ifndef MAIN_H
#define	MAIN_H

// FUNCTION PROTOTYPES
extern void I2S_init(void);
extern void DMA0_init(void);
extern void I2Sbuffer_init(void);
extern void DMA0_start(void);

void DCIInit(void);
//void processRxData(int * sourceBuffer, int * targetBuffer);
void processRxData(void);
//extern VectorCopy (int numElems, fractional* dstV, fractional* srcV);
//extern VectorAdd (int numElems, fractional* dstV, fractional* srcV1, fractional* srcV2);
void DMAInit(void);
void holdEngine(int *inBuff, int *outBuff, int *prevGrain, int ptrRdRAM, int ptrWrRAM, long int readPtr, long int writePtr, int fbk);

#endif	/* MAIN_H */

