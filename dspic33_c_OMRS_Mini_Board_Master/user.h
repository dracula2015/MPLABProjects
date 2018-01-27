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
/* TODO Application specific user parameters used in user.c may go here */
extern float globalTime;
extern long gloalTimeCount;
/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */
void InitApp(void);         /* I/O and Peripheral Initialization */
void UartInit(void);
void QEInit(void);
void PwmInit(void);
void TimerInit(void);

#endif