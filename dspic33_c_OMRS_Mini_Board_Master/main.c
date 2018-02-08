/* 
 * File:   main.c   
 * Author: dracula
 * Comments:
 * Revision history: EDITION 0.1 
 */

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file, thus actually include <p33FJ128MC804.h> */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__dsPIC33E__)
    	#include <p33Exxxx.h>
    #elif defined(__dsPIC33F__)
    	#include <p33Fxxxx.h>
    #endif
#endif

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */
#include <math.h>

#define PI 3.1415926
/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
/* i.e. uint16_t <variable_name>; */
Matrix *Jacobin;
Matrix *JConst;
Matrix *JCoeff;
bool go = 0;
bool stop = 0;
bool direction = 0;

int count[2]={0,0};
int motor = 0;
int i=0;
float loopTime=0.0;
float radius = 0.3;
float speed = PI / 15;
float rectLength = 1.0;
/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
int main(void)
{   
    /* Configure the oscillator for the device */
    ConfigureOscillator();
    /* Initialize IO ports and peripherals */
    InitApp();
    /* TODO <INSERT USER APPLICATION CODE HERE> */
    Jacobin = m_constructor(global, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    JCoeff = m_constructor(global, NULL, NULL, 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Kp = m_constructor(global, NULL, NULL, 6, 0, 0, 0, 6, 0, 0, 0, 6);
	Kd = m_constructor(global, NULL, NULL, 10, 0, 0, 0, 10, 0, 0, 0, 10);
    Vector3f* qd = v_constructor(global, NULL, 0, 0, 0);
	Vector3f* dqd = v_constructor(global, NULL, 0, 0, 0);
	Vector3f* ddqd = v_constructor(global, NULL, 0, 0, 0);
    Vector3f* qdPre = v_constructor(global, NULL, 0, 0, 0);
	Vector3f* dqdPre = v_constructor(global, NULL, 0, 0, 0);
    Vector3f* q = v_constructor(global, NULL, 0, 0, 0);
    Vector3f* qPre = v_constructor(global, NULL, 0, 0, 0);
	Vector3f* dq = v_constructor(global, NULL, 0, 0, 0);
    Vector3f* omega = v_constructor(global, NULL, 0, 0, 0);
    Vector3f* controlEffect;
//	Vector3f* ddq;
    P.m = 11.4;
	P.Iv = 0.65;
	P.r = 0.05;
	P.Din = 0.147;
	P.Dout = 0.236;
	//P.La = (P.Din + P.Dout) / 2;
	P.La = 0.2425;
	P.I0 = 6 * pow(10, -6);
	P.kt = 0.0208;
	//P.kb = 1 / 34.34;
	P.kb = 0.02076;
	P.n = 71;
	P.b0 = 6.0 * pow(10, -5);
	P.Ra = 1.53;
	P.beta0 = pow(P.n, 2) * P.I0 / pow(P.r, 2);
	P.beta1 = pow(P.n, 2) * (P.b0 + P.kt*P.kb / P.Ra) / pow(P.r, 2);
	P.beta2 = P.n*P.kt / P.r / P.Ra;
    Matrix* temp = m_constructor(local, NULL, NULL, -0.5, sqrt(3)/2, P.La, -0.5, -sqrt(3)/2, P.La, 1, 0, P.La);
    JConst = m_constructor(global, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    m_equal(JConst,m_inverse(temp));
    JCoeff->triMatrix[2][2] = P.r/P.n;

    float delta = 0.0;
//    float Jcoefficient=0.0;
    canTxMessage[0].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[0].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[0].buffer=0;
    canTxMessage[0].id=0x12345667;
    canTxMessage[0].data[0]=0x00;
    canTxMessage[0].data[1]=0x00;
    canTxMessage[0].data[2]=0x00;
    canTxMessage[0].data[3]=0x00;
    canTxMessage[0].data[4]=0x00;
    canTxMessage[0].data_length=5;
    
    canTxMessage[1].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[1].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[1].buffer=1;
    canTxMessage[1].id=0x12345668;
    canTxMessage[1].data[0]=0x00;
    canTxMessage[1].data[1]=0x00;
    canTxMessage[1].data[2]=0x00;
    canTxMessage[1].data[3]=0x00;
    canTxMessage[1].data[4]=0x00;
    canTxMessage[1].data_length=5;
    
    canTxMessage[2].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[2].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[2].buffer=2;
    canTxMessage[2].id=0x12345669;
    canTxMessage[2].data[0]=0x00;
    canTxMessage[2].data[1]=0x00;
    canTxMessage[2].data[2]=0x00;
    canTxMessage[2].data[3]=0x00;
    canTxMessage[2].data[4]=0x00;
    canTxMessage[2].data_length=5;
            
    canTxMessage[3].message_type=CAN_MSG_RTR;
    //canTxMessage.message_type=CAN_MSG_DATA;
    canTxMessage[3].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[3].buffer=6;
    canTxMessage[3].id=0x12344321;
    canTxMessage[3].data_length=4;
    
    //canTxMessage[4].message_type=CAN_MSG_RTR;
    canTxMessage[4].message_type=CAN_MSG_DATA;
    canTxMessage[4].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[4].buffer=7;
    canTxMessage[4].id=0x12312321;
    canTxMessage[4].data[0]=0x00;
    canTxMessage[4].data[1]=0x00;
    canTxMessage[4].data[2]=0x00;
    canTxMessage[4].data[3]=0x00;
    canTxMessage[4].data[4]=0x00;
    canTxMessage[4].data[5]=0x00;
    canTxMessage[4].data[6]=0x00;
    canTxMessage[4].data[7]=0x00;
    canTxMessage[4].data_length=8;
//    LATAbits.LATA8 = 1;
//    LATCbits.LATC0 = 1;
    while(1)
    {
        loopTime = globalTime;
        if(U1STAbits.PERR==1)
        {
            continue;
        }
        if(U1STAbits.OERR==1)
        {
            //LATAbits.LATA0=1;
            U1STAbits.OERR=0;
            //receivedNumber++;
            continue;
        }
        if(U1STAbits.URXDA==1)
        {   
            //LATAbits.LATA1=1;
        }
          
        if(i==0)
        {            
            motor = count[0];
            motor = motor & 0x00FF;
            motor = motor | (count[1]<<8);
        }
        
        //QEIPos = (QEIPosHigh << 16) + POS1CNT; 

        sendECAN(&canTxMessage[3]);
        /* there should be a delay here */
        /* there should be a delay here */
        /* there should be a delay here */
        Delay_Us(Delay200uS_count);
        Delay_Us(Delay200uS_count);
        /* check to see when a message is received and move the message 
		into RAM and parse the message */ 
//		if(canRxMessage[0].buffer_status==CAN_BUF_FULL)
        while(canRxMessage[0].buffer_status!=CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage[0]);			
			/* reset the flag when done */
			canRxMessage[0].buffer_status=CAN_BUF_EMPTY;
            wheelPos[0] = canRxMessage[0].data[0];
            wheelPos[0] = (wheelPos[0]<<8) + canRxMessage[0].data[1];
            wheelPos[0] = (wheelPos[0]<<8) + canRxMessage[0].data[2];
            wheelPos[0] = (wheelPos[0]<<8) + canRxMessage[0].data[3];
		}
//		if(canRxMessage[1].buffer_status==CAN_BUF_FULL)
        if(canRxMessage[1].buffer_status!=CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage[1]);			
			/* reset the flag when done */
			canRxMessage[1].buffer_status=CAN_BUF_EMPTY;
            wheelPos[1] = canRxMessage[1].data[0];
            wheelPos[1] = (wheelPos[1]<<8) + canRxMessage[1].data[1];
            wheelPos[1] = (wheelPos[1]<<8) + canRxMessage[1].data[2];
            wheelPos[1] = (wheelPos[1]<<8) + canRxMessage[1].data[3];
		}
//        if(canRxMessage[2].buffer_status==CAN_BUF_FULL)
        while(canRxMessage[2].buffer_status!=CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage[2]);			
			/* reset the flag when done */
			canRxMessage[2].buffer_status=CAN_BUF_EMPTY;
            wheelPos[2] = canRxMessage[2].data[0];
            wheelPos[2] = (wheelPos[2]<<8) + canRxMessage[2].data[1];
            wheelPos[2] = (wheelPos[2]<<8) + canRxMessage[2].data[2];
            wheelPos[2] = (wheelPos[2]<<8) + canRxMessage[2].data[3];
		};
        delta = globalTime -globalTimePre;
        if(stop){globalTime = 0.0;}
        globalTimePre = globalTime;
        
        qd->x = radius*cos(globalTime*speed);
		qd->y = radius*sin(globalTime*speed);
		qd->z = 0;
        dqd->x = -radius*speed*sin(globalTime*speed);
		dqd->y = radius*speed*cos(globalTime*speed);
		dqd->z = 0;
        ddqd->x = -radius*pow(speed,2)*cos(globalTime*speed);
		ddqd->y = -radius*pow(speed,2)*sin(globalTime*speed);
		ddqd->z = 0;
        
//        if(0<=fmodf(globalTime*speed,4*rectLength) && fmodf(globalTime*speed,4*rectLength)<rectLength)
//        {
//            qd->x = fmodf(globalTime*speed,4*rectLength);
//            qd->y = 0;
//        }
//        else if(rectLength<=fmodf(globalTime*speed,4*rectLength) && fmodf(globalTime*speed,4*rectLength)<2*rectLength)
//        {
//            qd->x = rectLength;
//            qd->y = fmodf(globalTime*speed,4*rectLength)-rectLength;
//        }
//        else if(2*rectLength<=fmodf(globalTime*speed,4*rectLength) && fmodf(globalTime*speed,4*rectLength)<3*rectLength)
//        {
//            qd->x = 3*rectLength-fmodf(globalTime*speed,4*rectLength);
//            qd->y = rectLength;
//        }
//        else if(3*rectLength<=fmodf(globalTime*speed,4*rectLength) && fmodf(globalTime*speed,4*rectLength)<4*rectLength)
//        {
//            qd->x = 0;
//            qd->y = 4*rectLength-fmodf(globalTime*speed,4*rectLength);
//        }
//        else;
//        if (globalTime > 15)
//        {
//            qd->z = 0.35 * (globalTime - 15);
//        }

//        v_equal(dqd,v_s_multiply(v_minus(qd,qdPre),1/delta));
//        v_equal(qdPre,qd);
//        v_equal(ddqd,v_s_multiply(v_minus(dqd,dqdPre),1/delta));
//        v_equal(dqdPre,dqd);
        
        for(i=0;i<3;i++)
        {
            wheelSpeed[i] = 2*PI*(wheelPos[i] - wheelPosPre[i])/2048/delta;
            wheelPosPre[i] = wheelPos[i];
        }
        omega->x = wheelSpeed[0];
        omega->y = wheelSpeed[1];
        omega->z = wheelSpeed[2];

//        Jcoefficient = P.r/(3*sqrt(3)*P.n*P.La);
//        Jacobin->triMatrix[0][0] = Jcoefficient * ( -2*P.La*sin(q->z + PI/3) - 2*P.La*sin(q->z) );
//        Jacobin->triMatrix[0][1] = Jcoefficient * ( 2*P.La*sin(q->z - PI/3) + 2*P.La*sin(q->z) );
//        Jacobin->triMatrix[0][2] = Jcoefficient * ( 2*P.La*sin(q->z + PI/3) + 2*P.La*sin(PI/3 - q->z) );
//        Jacobin->triMatrix[1][0] = Jcoefficient * ( 2*P.La*sin(PI/6 - q->z) + 2*P.La*cos(q->z) );
//        Jacobin->triMatrix[1][1] = Jcoefficient * ( -2*P.La*sin(q->z + PI/6) - 2*P.La*cos(q->z) );
//        Jacobin->triMatrix[1][2] = Jcoefficient * ( 2*P.La*sin(q->z - PI/6) + 2*P.La*sin(q->z + PI/6) );
//        Jacobin->triMatrix[2][0] = Jcoefficient * ( sqrt(3) );
//        Jacobin->triMatrix[2][1] = Jcoefficient * ( sqrt(3) );
//        Jacobin->triMatrix[2][2] = Jcoefficient * ( sqrt(3) );
        JCoeff->triMatrix[0][0] = P.r/P.n*cos(q->z);
        JCoeff->triMatrix[0][1] = -P.r/P.n*sin(q->z);
        JCoeff->triMatrix[1][0] = P.r/P.n*sin(q->z);
        JCoeff->triMatrix[1][1] = P.r/P.n*cos(q->z);
        m_equal(Jacobin,m_m_multiply(JCoeff,JConst));

        v_equal(dq,m_v_multiply(Jacobin,omega));
        
        q->x = qPre->x + dq->x * delta;
        q->y = qPre->y + dq->y * delta;
        q->z = qPre->z + dq->z * delta;
        
        qPre->x = q->x;
        qPre->y = q->y;
        qPre->z = q->z;
        
        controlEffect = OMRS_controller(qd, dqd, ddqd, q, dq);
        
        if(stop){
            canTxMessage[0].data[4] = 0;
            canTxMessage[1].data[4] = 0;
            canTxMessage[2].data[4] = 0;
        }
        
        if(go){
            canTxMessage[0].data[4] = 1;
            canTxMessage[1].data[4] = 1;
            canTxMessage[2].data[4] = 1;
        }
        
        canTxMessage[0].data[0] =  ((long)(100*controlEffect->x))>>24;
        canTxMessage[0].data[1] =  ((long)(100*controlEffect->x))>>16;
        canTxMessage[0].data[2] =  ((long)(100*controlEffect->x))>>8;
        canTxMessage[0].data[3] =  ((long)(100*controlEffect->x));
        
        canTxMessage[1].data[0] =  ((long)(100*controlEffect->y))>>24;
        canTxMessage[1].data[1] =  ((long)(100*controlEffect->y))>>16;
        canTxMessage[1].data[2] =  ((long)(100*controlEffect->y))>>8;
        canTxMessage[1].data[3] =  ((long)(100*controlEffect->y));
        
        canTxMessage[2].data[0] =  ((long)(100*controlEffect->z))>>24;
        canTxMessage[2].data[1] =  ((long)(100*controlEffect->z))>>16;
        canTxMessage[2].data[2] =  ((long)(100*controlEffect->z))>>8;
        canTxMessage[2].data[3] =  ((long)(100*controlEffect->z));
        
        sendECAN(&canTxMessage[0]);
        sendECAN(&canTxMessage[1]);
        sendECAN(&canTxMessage[2]);
        
        /* there should be a delay here */
        /* there should be a delay here */
        /* there should be a delay here */
        
        /* release dynamically allocated local memory */
        freeLocalMem();

        U1TXREG = (long)((globalTime - loopTime)*10000);      
        U1TXREG = (long)((globalTime - loopTime)*10000)>>8;   
    };
    /* release dynamically allocated global memory */
    freeGlobalMem();
    
    return 0;
}
