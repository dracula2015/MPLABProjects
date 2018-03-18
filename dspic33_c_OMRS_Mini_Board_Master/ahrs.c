/*
 * File:   ahrs.c
 * Author: dracula
 *
 * Created on March 16, 2018, 11:16 AM
 */

#include "user.h"

void ahrs_decode(unsigned int *ahrsSignal,float *ahrsAttitude, float *ahrsAcclerom)
{
    int attitudeCount = 2;
    int acceleromCount = 2;
    int signal = 23;//23;
    long attitude[3] = {0,0,0};
    long accelerom[3] = {0,0,0};
    unsigned char order = 0;
    char realOrder = 0;
    long mantissa = 0;
    
    for(acceleromCount=2;acceleromCount>=0;acceleromCount--)
    {
        accelerom[acceleromCount] = ahrsSignal[signal];
        signal--;
        accelerom[acceleromCount] = (accelerom[acceleromCount]<<8) + ahrsSignal[signal];
        signal--;
        accelerom[acceleromCount] = (accelerom[acceleromCount]<<8) + ahrsSignal[signal];
        signal--;
        accelerom[acceleromCount] = (accelerom[acceleromCount]<<8) + ahrsSignal[signal];
        signal--;
        order = accelerom[acceleromCount]>>23;
        realOrder = order - 127;
        mantissa = (accelerom[acceleromCount] & 0x7FFFFF) + 0x800000;
        ahrsAcclerom[acceleromCount] = mantissa * pow(2,realOrder-23);
        if(accelerom[acceleromCount] & 0x80000000)
        {
            ahrsAcclerom[acceleromCount] = - ahrsAcclerom[acceleromCount];
        }
    }
    
    for(attitudeCount=2;attitudeCount>=0;attitudeCount--)
    {
        attitude[attitudeCount] = ahrsSignal[signal];
        signal--;
        attitude[attitudeCount] = (attitude[attitudeCount]<<8) + ahrsSignal[signal];
        signal--;
        attitude[attitudeCount] = (attitude[attitudeCount]<<8) + ahrsSignal[signal];
        signal--;
        attitude[attitudeCount] = (attitude[attitudeCount]<<8) + ahrsSignal[signal];
        signal--;
        order = attitude[attitudeCount]>>23;
        realOrder = order - 127;
        mantissa = (attitude[attitudeCount] & 0x7FFFFF) + 0x800000;
        ahrsAttitude[attitudeCount] = mantissa * pow(2,realOrder-23);
        if(attitude[attitudeCount] & 0x80000000)
        {
            ahrsAttitude[attitudeCount] = - ahrsAttitude[attitudeCount];
        }
    }
}