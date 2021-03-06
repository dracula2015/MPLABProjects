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

#define PI 3.1415926
#define UART1_MSG_BUF_LENGTH 2
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

typedef struct Command
{
    union {
        unsigned char signal[12];
        float command[3];
    };
} MATCOMMAND;

typedef struct QEISpeed
{
    union {
        unsigned char signal[12];
        float speed[3];
    };
} QEISpeed;

typedef unsigned char UART1MSGBUF [UART1_MSG_BUF_LENGTH][13];
extern UART1MSGBUF uart1MsgBuf __attribute__((space(dma)));

extern Matrix *Jacobin;
extern Matrix *JConst;
extern Matrix *JCoeff;
extern Matrix* JBackMatrix;
//extern Matrix* WMobile;
extern Vector3f* qd;
extern Vector3f* dqd;
extern Vector3f* ddqd;
extern Vector3f* qdPre;
extern Vector3f* dqdPre;
extern Vector3f* q;
extern Vector3f* qPre;
extern Vector3f* dq;
extern Vector3f* omega;
extern Vector3f* controlEffect;
extern Vector3f* joystick;
extern Vector3f* joystickError;
extern Vector3f* ahrsAttitude;
extern Vector3f* ahrsAttitudePre;
extern Vector3f* dAhrsAttitude;

extern bool go;
extern bool stop;
extern bool direction;
extern bool reset;

extern float joystickGainKP;
extern float joystickGainKD;
extern float debounceTime;
extern float eliminateJitter;
extern bool debounce;
extern bool debounceEdge;

extern int count[6];
extern int motor[3];
extern int hostCommandCount;
extern float radius;
extern float speed;
extern float rectLength;

extern float loopTime;
extern float globalTime;
extern float globalTimePre;
extern float delta;
extern unsigned char ReceivedChar, ReceivedChar1;
extern unsigned char TransmitChar, TransmitChar1;
extern unsigned int radioSignal[25];
extern int radioChannel[16];
extern float radioInterval;
extern unsigned int radioSignalCount;
//extern unsigned int ahrsSignalCount;
//extern unsigned char ahrsSignal[24];
//extern float ahrsAttitude[3];
//extern float ahrsAcclerom[3];

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
extern AHRS ahrs;
extern MATCOMMAND matlabVoltage;
extern QEISpeed qeiSpeed;
extern Matrix *Kp;
extern Matrix *Kd;
extern bool QEIStatus;
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
void cfgDma5UartRx(void);
void cfgDma4UartTx(void);
void InitialParameters(void);
void ahrs_decode(unsigned int *ahrsSignal,float *ahrsAttitude, float *ahrsAcclerom);
void sbus_decode(unsigned int *radioSignal,int *radioChannel);
void Debounce(void);
void Joystick(void);
void Trajectory(void);
#endif
