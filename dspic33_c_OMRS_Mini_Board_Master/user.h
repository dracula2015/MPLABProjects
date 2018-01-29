/* 
 * File:   user.h   
 * Author: dracula
 * Comments:
 * Revision history: EDITION 0.1 
 */

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/
// This is a guard condition so that contents of this file are not included more than once.  
#ifndef XC_HEADER_USER_H
#define	XC_HEADER_USER_H

#include "matrix.h"
#include "vector.h"

#include "delay.h"
#include "ecan.h"

/* TODO Application specific user parameters used in user.c may go here */
typedef struct Parameter
{
	float m;
	float Iv;
	float r;
	float Din;
	float Dout;
	float La;
	float I0;
	float kt;
	float kb;
	float n;
	float b0;
	float Ra;
	float beta0;
	float beta1;
	float beta2;
}Parameter;

extern float globalTime;
extern float globalTimePre;
extern char ReceivedChar;
extern char TransmitChar;
extern long QEIPos;
extern long QEIPosHigh;
extern long wheelPos[3];
extern long wheelPosPre[3];
extern float wheelSpeed[3];

extern int countMatrix;
extern int countVector;
extern int countMatrixGlobal;
extern int countVectorGlobal;

extern Matrix* pointerMatrix[100];
extern Matrix* pointerMatrixGlobal[100];
extern Vector3f* pointerVector[100];
extern Vector3f* pointerVectorGlobal[100];

extern Parameter P;
extern Matrix *Kp;
extern Matrix *Kd;
/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/
/* TODO User level functions prototypes (i.e. InitApp) go here */
Vector3f *OMRS_controller(Vector3f *qd, Vector3f *dqd, Vector3f *ddqd, Vector3f *q, Vector3f *dq);
Vector3f *OMRS_model(Vector3f *u, Vector3f *q, Vector3f *dq);

void InitApp(void);         /* I/O and Peripheral Initialization */
void UartInit(void);
void QEInit(void);
void PwmInit(void);
void TimerInit(void);

#endif