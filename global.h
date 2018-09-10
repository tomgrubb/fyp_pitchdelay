#ifndef GLOBAL_H
#define	GLOBAL_H

// DEFINE GLOBAL VARIABLES

int tempBuffer[FRAME];
int testValue = 0;

int count = 0;      // counter for delay engine startup
int flagRX = 0;     // flag for RX buffer full
int flagTX = 0;     // flag for TX buffer empty
int buffer = 1;     // indicator for ping-pong buffer target
int reverse = 0;    // enable variable for reverse delay

int fbkA = 0x4000;  // feedback coefficient for delay A
int fbkB = 0x4000;
int lvlA = 0x8000;  // level coefficient for delay A
int lvlB = 0x8000;
unsigned long timeA = maxPtr;
int maxMemA = 0;

// pointers to selected buffer
int *rxPtr;
int *txPtr;

//int *buffPtrA;
//int *buffPtrB;
//int *feedPtr;

// individual buffers for each delay tap
int bufferA[FRAME];
int bufferB[FRAME];

// buffer for feedback
int feedback[FRAME];

// buffer for previous grain
int holdBuffer[FRAME];

// cross fade variables
int crossScaleA[4] = { 0x1000, 0x4000, 0x6000, 0x7000 };
int crossScaleB[4] = { 0x7000, 0x6000, 0x4000, 0x1000 };
int indexC = 0;
long int crossCount = 0;
int dir = 1;

// DELAY POINTERS
long int readPtrA = 50*FRAME;      // read pointer for buffer A
long int writePtrA = 0;            // write pointer for buffer A
int RAM_ReadPtrA = 0;              // RAM pointer for reading A
int RAM_WritePtrA = 0;             // RAM pointer for writing A

long int readPtrB = 0;             // read pointer for buffer B
long int writePtrB = (60*FRAME);   // write pointer for buffer B
int RAM_WritePtrB = 2;             // RAM pointer for reading B
int RAM_ReadPtrB = 2;              // RAM pointer for writing B

//    int testDataIn[FRAME] = { 0xFFAA, 0xAAFF, 0xFAFA };
//    int testDataOut[FRAME];

#endif /* GLOBAL_H */