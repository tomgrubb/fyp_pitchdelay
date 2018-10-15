#ifndef GLOBAL_H
#define	GLOBAL_H

// DEFINE GLOBAL VARIABLES

int parameters[15];

int zDC[2] = {0,0}; // delayed values xz1 and yz1 for dc blocker
int xz1 = 0;
int yz1 = 0;

int tempBuffer[FRAME];
int testValue = 0;

int count = 0;      // counter for delay engine startup
int flagRX = 0;     // flag for RX buffer full
int flagTX = 0;     // flag for TX buffer empty
int buffer = 1;     // indicator for ping-pong buffer target
int reverse = 0;    // enable variable for reverse delay

// Feedback coefficients for Delay Line A, B and C
int fbkA = 0x4000;
int fbkB = 0x4000;
int fbkC = 0x4000;

// Level coefficients for Delay Line A, B and C
int lvlA = 0x8000;
int lvlB = 0x8000;
int lvlC = 0x4000;

// delay line enables
int armA = 0;
int armB = 0;
int armC = 0;

int indexTimeA = 50;
unsigned long timeA = 24500;
long int nextTimeA = 0;

unsigned long timeB = 24500;
long int nextTimeB = 0;

unsigned long timeC = 24500;
long int nextTimeC = 0;

long int targTimeA = 0;
int fade = 0;

char maxMem[3] = {1, 2, 3};
char memStart[3] = {1, 2, 3};

int timeCount = 0;

typedef union float2bytes { float f; char b[sizeof(float)];} f2b;
f2b masterTime;
int lock = 0;

// individual buffers for each delay tap
int bufferA[FRAME];
int bufferB[FRAME];

// buffer for feedback
int feedback[FRAME];

// buffer for previous grain
int holdBuffer[FRAME];

// cross fade variables
int indexC = 0;
long int crossCount = 0;
int dir = 1;
int mod = 0;
long int modCount = 0;
long int changeCount = 0;

// DELAY POINTERS
long int readPtrA = 0;             // read pointer for buffer A
long int writePtrA = (300*FRAME);  // write pointer for buffer A
int RAM_ReadPtrA = 1;              // RAM pointer for reading A
int RAM_WritePtrA = 1;             // RAM pointer for writing A

long int readPtrB = 0;             // read pointer for buffer B
long int writePtrB = (100*FRAME);  // write pointer for buffer B
int RAM_WritePtrB = 2;             // RAM pointer for reading B
int RAM_ReadPtrB = 2;              // RAM pointer for writing B

long int readPtrC = 0;             // read pointer for buffer C
long int writePtrC = (200*FRAME);  // write pointer for buffer C
int RAM_WritePtrC = 2;             // RAM pointer for reading C
int RAM_ReadPtrC = 2;              // RAM pointer for writing C


#endif /* GLOBAL_H */