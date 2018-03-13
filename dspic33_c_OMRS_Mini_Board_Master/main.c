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

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
/* i.e. uint16_t <variable_name>; */

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
    InitialParameters();

    int wheeli = 0;
//    float Jcoefficient = 0.0;
//    LATAbits.LATA8 = 1;
//    LATCbits.LATC0 = 1;
//    LATCbits.LATC5 = 1;
    while(1)
    {
        loopTime = globalTime;

        //QEIPos = (QEIPosHigh << 16) + POS1CNT; 

        sendECAN(&canTxMessage[3]);

        /* check to see when a message is received and move the message 
		into RAM and parse the message */ 
		if(canRxMessage[0].buffer_status==CAN_BUF_FULL)
//        while(canRxMessage[0].buffer_status!=CAN_BUF_FULL);
		{
			rxECAN(&canRxMessage[0]);			
			/* reset the flag when done */
			canRxMessage[0].buffer_status=CAN_BUF_EMPTY;
            wheelPos[0] = canRxMessage[0].data[0];
            wheelPos[0] = (wheelPos[0]<<8) + canRxMessage[0].data[1];
            wheelPos[0] = (wheelPos[0]<<8) + canRxMessage[0].data[2];
            wheelPos[0] = (wheelPos[0]<<8) + canRxMessage[0].data[3];
		}
		if(canRxMessage[1].buffer_status==CAN_BUF_FULL)
//        while(canRxMessage[1].buffer_status!=CAN_BUF_FULL);
		{
			rxECAN(&canRxMessage[1]);			
			/* reset the flag when done */
			canRxMessage[1].buffer_status=CAN_BUF_EMPTY;
            wheelPos[1] = canRxMessage[1].data[0];
            wheelPos[1] = (wheelPos[1]<<8) + canRxMessage[1].data[1];
            wheelPos[1] = (wheelPos[1]<<8) + canRxMessage[1].data[2];
            wheelPos[1] = (wheelPos[1]<<8) + canRxMessage[1].data[3];
		}
        if(canRxMessage[2].buffer_status==CAN_BUF_FULL)
//        while(canRxMessage[2].buffer_status!=CAN_BUF_FULL);
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
        if(reset)
        {
            canTxMessage[0].data[5] = 1;
            canTxMessage[1].data[5] = 1;
            canTxMessage[2].data[5] = 1;
            
            q->x = 0;
            qPre->x = 0;
            q->y = 0;
            qPre->y = 0;
            q->z = 0;
            qPre->z = 0;
            for(wheeli=0;wheeli<3;wheeli++)
            {
                wheelPos[wheeli] = 0;
                wheelPosPre[wheeli] = 0;
            }
            globalTime = 0.0;
            reset = false;
        }else
        {
            canTxMessage[0].data[5] = 0;
            canTxMessage[1].data[5] = 0;
            canTxMessage[2].data[5] = 0;
        }
        globalTimePre = globalTime;
        
        Trajectory();
                
        for(wheeli=0;wheeli<3;wheeli++)
        {
            wheelSpeed[wheeli] = 2*PI*(wheelPos[wheeli] - wheelPosPre[wheeli])/2048/delta;
            wheelPosPre[wheeli] = wheelPos[wheeli];
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
        
        Joystick();
        
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
        Delay_Us(Delay200uS_count);
        Delay_Us(Delay200uS_count);
        
        /* release dynamically allocated local memory */
        freeLocalMem();
//        U1TXREG = 'u';
//        U1TXREG = (long)((globalTime - loopTime)*10000)>>8;  
//        U1TXREG = (long)((globalTime - loopTime)*10000);      
    };
    /* release dynamically allocated global memory */
    freeGlobalMem();
    
    return 0;
}