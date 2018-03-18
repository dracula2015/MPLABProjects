/* 
 * File:   user.h   
 * Author: dracula
 * Comments:
 * Revision history: EDITION 0.1 
 */

// This is a guard condition so that contents of this file are not included more than once.  
#ifndef XC_HEADER_USER_H
#define	XC_HEADER_USER_H

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/
#include "delay.h"
#include "ecan.h"
#include <stdint.h>        /* Includes int definition                         */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <math.h>
/* TODO Application specific user parameters used in user.c may go here */

typedef struct Ahrs
{
    union {
        unsigned char signal[24];
        struct {
            float attitude[3];
            float acclerom[3];
        };
    };
} AHRS;

extern float globalTime;
extern char ReceivedChar, ReceivedChar1;
extern char TransmitChar, TransmitChar1;
extern long QEIPos;
extern long QEIPosHigh;

//extern unsigned char ahrsSignal[24];
//extern float ahrsAttitude[3];
//extern float ahrsAcclerom[3];
//extern unsigned char* attitudeAddr;
//extern unsigned int radioSignalCount;
extern unsigned int ahrsSignalCount;
extern AHRS ahrs;
extern bool ahrsMessage;
/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */
void InitApp(void);         /* I/O and Peripheral Initialization */
void UartInit(void);
void QEInit(void);
void PwmInit(void);
void TimerInit(void);
void ahrs_decode(unsigned int *ahrsSignal,float *ahrsAttitude, float *ahrsAcclerom);
#endif