/*
 * File:   filtering.c
 * Author: tom
 *
 * Created on September 17, 2018, 10:35 AM
 */


#include "xc.h"
#include "MACROS.h"
#include "dsp.h"

// DC blocking input filter
void blockDC(int *inPtr, int *outPtr, int *xz, int *yz)
{
    int n;  
    int temp = 0;
    
    for (n = 0; n < FRAME; n++)
    {   
        temp = (int)yz;
        temp *= 0.9;
        temp -= (int)xz;
        outPtr[n] = inPtr[n] + temp;
        *yz = outPtr[n];
        *xz = inPtr[n];
    }
}
