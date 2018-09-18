#ifndef GLOBAL_H
#define	GLOBAL_H

// DEFINE GLOBAL VARIABLES

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
int fadeCount = 0;


int fbkA = 0x4000;  // feedback coefficient for delay A
int fbkB = 0x4000;
int lvlA = 0x8000;  // level coefficient for delay A
int lvlB = 0x8000;
int indexTimeA = 50;
unsigned long timeA = 24500;
long int nextTimeA = 0;
long int targTimeA = 0;
int fade = 0;
int potVal = 100;

char maxMem[3] = {1, 2, 3};
char memStart[3] = {1, 2, 3};

int timeCount = 0;

// pointers to selected buffer
int *rxPtr;
int *txPtr;

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
long int cFadePtrA = 0;            // crossfade pointer for time change 
long int readPtrA = 0;             // read pointer for buffer A
long int writePtrA = (200*FRAME);  // write pointer for buffer A
int RAM_ReadPtrA = 1;              // RAM pointer for reading A
int RAM_WritePtrA = 1;             // RAM pointer for writing A
int RAM_FadePtrA = 1;

long int readPtrB = 0;             // read pointer for buffer B
long int writePtrB = (60*FRAME);   // write pointer for buffer B
int RAM_WritePtrB = 1;             // RAM pointer for reading B
int RAM_ReadPtrB = 1;              // RAM pointer for writing B

#endif /* GLOBAL_H */