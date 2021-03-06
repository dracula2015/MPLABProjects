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

    int ahrsCount = 0;
    int thetaCount =0;
    float attitudePre = 0.0;
    int wheeli = 0;
//    LATAbits.LATA8 = 1;
//    LATCbits.LATC0 = 1;
//    LATCbits.LATC5 = 1;
    while(1)
    {
        loopTime = globalTime;

        //QEIPos = (QEIPosHigh << 16) + POS1CNT; 

//        if((radioChannel[4]==0x01E0) || (radioChannel[4]==0x0400)) 
//        {
//            sendECAN(&canTxMessage[3]);
//        }
//
//        /* check to see when a message is received and move the message 
//		into RAM and parse the message */ 
//		if(canRxMessage[0].buffer_status==CAN_BUF_FULL)
////        while(canRxMessage[0].buffer_status!=CAN_BUF_FULL);
//		{
//			rxECAN(&canRxMessage[0]);			
//			/* reset the flag when done */
//			canRxMessage[0].buffer_status=CAN_BUF_EMPTY;
//            wheelPos[0] = canRxMessage[0].data[0];
//            wheelPos[0] = (wheelPos[0]<<8) + canRxMessage[0].data[1];
//            wheelPos[0] = (wheelPos[0]<<8) + canRxMessage[0].data[2];
//            wheelPos[0] = (wheelPos[0]<<8) + canRxMessage[0].data[3];
//		}
//		if(canRxMessage[1].buffer_status==CAN_BUF_FULL)
////        while(canRxMessage[1].buffer_status!=CAN_BUF_FULL);
//		{
//			rxECAN(&canRxMessage[1]);			
//			/* reset the flag when done */
//			canRxMessage[1].buffer_status=CAN_BUF_EMPTY;
//            wheelPos[1] = canRxMessage[1].data[0];
//            wheelPos[1] = (wheelPos[1]<<8) + canRxMessage[1].data[1];
//            wheelPos[1] = (wheelPos[1]<<8) + canRxMessage[1].data[2];
//            wheelPos[1] = (wheelPos[1]<<8) + canRxMessage[1].data[3];
//		}
//        if(canRxMessage[2].buffer_status==CAN_BUF_FULL)
////        while(canRxMessage[2].buffer_status!=CAN_BUF_FULL);
//		{
//			rxECAN(&canRxMessage[2]);			
//			/* reset the flag when done */
//			canRxMessage[2].buffer_status=CAN_BUF_EMPTY;
//            wheelPos[2] = canRxMessage[2].data[0];
//            wheelPos[2] = (wheelPos[2]<<8) + canRxMessage[2].data[1];
//            wheelPos[2] = (wheelPos[2]<<8) + canRxMessage[2].data[2];
//            wheelPos[2] = (wheelPos[2]<<8) + canRxMessage[2].data[3];
//		};
//        if(canRxMessage[3].buffer_status==CAN_BUF_FULL)
//		{
//			rxECAN(&canRxMessage[3]);			
//			/* reset the flag when done */
//			canRxMessage[3].buffer_status=CAN_BUF_EMPTY;
//            for(ahrsCount=0;ahrsCount<8;ahrsCount++)
//            {
//                ahrs.signal[ahrsCount] = canRxMessage[3].data[ahrsCount];
//            }
//            if(ahrs.attitude[0] - attitudePre > PI)
//            {
//                thetaCount--;
//            }else if(ahrs.attitude[0] - attitudePre < -PI)
//            {
//                thetaCount++;
//            }
//            attitudePre = ahrs.attitude[0];
//            ahrsAttitude->z = - (2*thetaCount*PI + ahrs.attitude[0]);
////            if(radioChannel[11]==0x0160)
////            {
////                ahrsAttitude->z = - (2*thetaCount*PI + ahrs.attitude[0]);
////            }else
////            {
////                ahrsAttitude->z = - ahrs.attitude[0];
////            }
//		}
//        if(canRxMessage[4].buffer_status==CAN_BUF_FULL)
//		{
//			rxECAN(&canRxMessage[4]);			
//			/* reset the flag when done */
//			canRxMessage[4].buffer_status=CAN_BUF_EMPTY;
//            for(ahrsCount=8;ahrsCount<16;ahrsCount++)
//            {
//                ahrs.signal[ahrsCount] = canRxMessage[4].data[ahrsCount-8];
//            }
//            dAhrsAttitude->x = ahrs.acclerom[0];
//		}
//        if(canRxMessage[5].buffer_status==CAN_BUF_FULL)
//		{
//			rxECAN(&canRxMessage[5]);			
//			/* reset the flag when done */
//			canRxMessage[5].buffer_status=CAN_BUF_EMPTY;
//            for(ahrsCount=16;ahrsCount<24;ahrsCount++)
//            {
//                ahrs.signal[ahrsCount] = canRxMessage[5].data[ahrsCount-16];
//            }
//            dAhrsAttitude->y = ahrs.acclerom[1];
//		}
//        
//        delta = globalTime -globalTimePre;
//        globalTimePre = globalTime;
//        if(stop){globalTime = 0.0;globalTimePre = -0.001;}       
//        if(reset)
//        {
//            canTxMessage[0].data[5] = 1;
//            canTxMessage[1].data[5] = 1;
//            canTxMessage[2].data[5] = 1;
//            
//            q->x = 0;
//            qPre->x = 0;
//            q->y = 0;
//            qPre->y = 0;
//            q->z = 0;
//            qPre->z = 0;
//            for(wheeli=0;wheeli<3;wheeli++)
//            {
//                wheelPos[wheeli] = 0;
//                wheelPosPre[wheeli] = 0;
//            }
//            controlEffect->x = 0;
//            controlEffect->y = 0;
//            controlEffect->z = 0;
//            globalTime = 0.0;
//            globalTimePre = -0.001;
//            reset = false;
//        }else
//        {
//            canTxMessage[0].data[5] = 0;
//            canTxMessage[1].data[5] = 0;
//            canTxMessage[2].data[5] = 0;
//        }
////        if((radioChannel[4]==0x0160) || (radioChannel[4]==0x0400)) 
//        {
//            for(wheeli=0;wheeli<3;wheeli++)
//            {
//                wheelSpeed[wheeli] = 2*PI*(wheelPos[wheeli] - wheelPosPre[wheeli])/2048/delta;
//                wheelPosPre[wheeli] = wheelPos[wheeli];
//            }
//            omega->x = wheelSpeed[0];
//            omega->y = wheelSpeed[1];
//            omega->z = wheelSpeed[2];
//        }
////        if(radioChannel[4]==0x01E0)
//        {
//            Trajectory();
//
//            JCoeff->triMatrix[0][0] = P.r/P.n*cos(q->z);
//            JCoeff->triMatrix[0][1] = -P.r/P.n*sin(q->z);
//            JCoeff->triMatrix[1][0] = P.r/P.n*sin(q->z);
//            JCoeff->triMatrix[1][1] = P.r/P.n*cos(q->z);
//            JCoeff->triMatrix[2][2] = P.r/P.n;
//            m_equal(Jacobin,m_m_multiply(JCoeff,JConst));
//
//            v_equal(dq,m_v_multiply(Jacobin,omega));
//            
////            bool QEIStatus = false;
//            qeiSpeed.speed[0] = dq->x;
//            qeiSpeed.speed[1] = dq->y;
//            qeiSpeed.speed[2] = dq->z;
//            int i=0;
//            for(i=0;i<12;i++)
//            {
//                uart1MsgBuf[0][i] = qeiSpeed.signal[i];
//            }
//            uart1MsgBuf[0][12] = 0x0A;
//            
////            DMA4CONbits.CHEN  = 1;			// Re-enable DMA0 Channel
////            DMA4REQbits.FORCE = 1;			// Manual mode: Kick-start the first transfer
////            bool QEIStatus = true;
//
//            q->x = qPre->x + dq->x * delta;
//            q->y = qPre->y + dq->y * delta;
//            q->z = qPre->z + dq->z * delta;
//
//            qPre->x = q->x;
//            qPre->y = q->y;
//            qPre->z = q->z;
//        }
//
//        ahrsAttitude->x = ahrsAttitude->x + dAhrsAttitude->x * delta;
//        ahrsAttitude->y = ahrsAttitude->y + dAhrsAttitude->y * delta;
//        
////        controlEffect = OMRS_controller(qd, dqd, ddqd, q, dq);
//        
        Joystick();
        
        sendECAN(&canTxMessage[0]);
        sendECAN(&canTxMessage[1]);
        sendECAN(&canTxMessage[2]);
        
        /* there should be a delay here */
        Delay_Us(Delay200uS_count);
        Delay_Us(Delay200uS_count);
        
        /* release dynamically allocated local memory */
        freeLocalMem();
//        U1TXREG = 'u';
//        U1TXREG = (long)((globalTime - loopTime)*10000)>>16;   
//        U1TXREG = (long)((globalTime - loopTime)*10000)>>8;  
//        U1TXREG = (long)((globalTime - loopTime)*10000);      
    };
    /* release dynamically allocated global memory */
    freeGlobalMem();
    
    return 0;
}