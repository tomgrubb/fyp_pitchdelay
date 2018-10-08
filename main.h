#ifndef MAIN_H
#define	MAIN_H

int paramAdd[] =
{
    0xA0, 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC, 0xAE,
    0xB0, 0xB2, 0xB4, 0xB6, 0xB8, 0xBA, 0xBC, 0xBE,
};

// FUNCTION PROTOTYPES
extern void I2S_init(void);
extern void DMA0_init(void);
extern void I2Sbuffer_init(void);
extern void DMA0_start(void);

void DCIInit(void);
void processRxData(void);
void DMAInit(void);
void holdEngine(int *inBuff, int *outBuff, int *prevGrain, int ptrRdRAM, int ptrWrRAM, long int readPtr, long int writePtr, int fbk);

#endif	/* MAIN_H */

