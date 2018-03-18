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
#include <stdbool.h>
//#include <p33FJ128MC804.h>       /* Includes true/false definition                  */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
/* i.e. uint16_t <variable_name>; */

bool go = 0;
bool stop = 1;
bool direction = 0;
long controlEffect = 0;
int motor = 0;

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

    int ahrsCount = 0;
    int i = 0;
    for(i=0;i<24;i++)
    {
        ahrs.signal[i] = 0;
    }
    ahrs.attitude[0] = 0.0;
    ahrs.attitude[1] = 0.0;
    ahrs.attitude[2] = 0.0;
    ahrs.acclerom[0] = 0.0;
    ahrs.acclerom[1] = 0.0;
    ahrs.acclerom[2] = 0.0;
    
    /* configure and send a message */
    canTxMessage[0].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[0].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[0].buffer=0;
    canTxMessage[0].id=0x12345677;
    canTxMessage[0].data_length=8;

    QEIPos = (QEIPosHigh << 16) + POS1CNT; 
    canTxMessage[0].data[0] = QEIPos >> 24;
    canTxMessage[0].data[1] = QEIPos >> 16;
    canTxMessage[0].data[2] = QEIPos >> 8;
    canTxMessage[0].data[3] = QEIPos;
    canTxMessage[0].data[4] = 0;
    canTxMessage[0].data[5] = 0;
    canTxMessage[0].data[6] = 0;
    canTxMessage[0].data[7] = 0;
    ecanRtrRespond(&canTxMessage[0]);
    /* send a CAN message */
    sendECAN(&canTxMessage[0]);

    /* configure and send a message */
    canTxMessage[1].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[1].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[1].buffer=4;
    canTxMessage[1].id=0x12345657;
    canTxMessage[1].data_length=8;
    canTxMessage[1].data[0] = 0;
    canTxMessage[1].data[1] = 0;
    canTxMessage[1].data[2] = 0;
    canTxMessage[1].data[3] = 0;
    canTxMessage[1].data[4] = 0;
    canTxMessage[1].data[5] = 0;
    canTxMessage[1].data[6] = 0;
    canTxMessage[1].data[7] = 0;
    
    /* configure and send a message */
    canTxMessage[2].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[2].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[2].buffer=5;
    canTxMessage[2].id=0x12345658;
    canTxMessage[2].data_length=8;
    canTxMessage[2].data[0] = 0;
    canTxMessage[2].data[1] = 0;
    canTxMessage[2].data[2] = 0;
    canTxMessage[2].data[3] = 0;
    canTxMessage[2].data[4] = 0;
    canTxMessage[2].data[5] = 0;
    canTxMessage[2].data[6] = 0;
    canTxMessage[2].data[7] = 0;
    
    /* configure and send a message */
    canTxMessage[3].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[3].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[3].buffer=6;
    canTxMessage[3].id=0x12345659;
    canTxMessage[3].data_length=8;
    canTxMessage[3].data[0] = 0;
    canTxMessage[3].data[1] = 0;
    canTxMessage[3].data[2] = 0;
    canTxMessage[3].data[3] = 0;
    canTxMessage[3].data[4] = 0;
    canTxMessage[3].data[5] = 0;
    canTxMessage[3].data[6] = 0;
    canTxMessage[3].data[7] = 0;

//    Delay_Us(Delay200uS_count);
    Delay(Delay_1S_Cnt);
    U2TXREG = '#';
    U2TXREG = 'f';
    while(1)
    {
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
        
        QEIPos = (QEIPosHigh << 16) + POS1CNT; 
        canTxMessage[0].data[0] = QEIPos >> 24;
        canTxMessage[0].data[1] = QEIPos >> 16;
        canTxMessage[0].data[2] = QEIPos >> 8;
        canTxMessage[0].data[3] = QEIPos;
        canTxMessage[0].data[4] = ahrs.signal[0];
        canTxMessage[0].data[5] = ahrs.signal[1];
        canTxMessage[0].data[6] = ahrs.signal[2];
        canTxMessage[0].data[7] = ahrs.signal[3];
        ecanRtrRespond(&canTxMessage[0]);
 
        if(ahrsMessage)
        {
            for(ahrsCount=0;ahrsCount<8;ahrsCount++)
            {
                canTxMessage[1].data[ahrsCount] = ahrs.signal[ahrsCount];
            }
            
            for(ahrsCount=8;ahrsCount<16;ahrsCount++)
            {
                canTxMessage[2].data[ahrsCount-8] = ahrs.signal[ahrsCount];
            }
            
            for(ahrsCount=16;ahrsCount<24;ahrsCount++)
            {
                canTxMessage[3].data[ahrsCount-16] = ahrs.signal[ahrsCount];
            }
            
            sendECAN(&canTxMessage[1]);
            sendECAN(&canTxMessage[2]);
            sendECAN(&canTxMessage[3]);
            ahrsMessage = false;
        }
        
        /* check to see when a message is received and move the message 
		into RAM and parse the message */ 
		if(canRxMessage[0].buffer_status==CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage[0]);			
			/* reset the flag when done */
			canRxMessage[0].buffer_status=CAN_BUF_EMPTY;
            controlEffect = canRxMessage[0].data[0];
            controlEffect = (controlEffect<<8) + canRxMessage[0].data[1];
            controlEffect = (controlEffect<<8) + canRxMessage[0].data[2];
            controlEffect = (controlEffect<<8) + canRxMessage[0].data[3];
            
            if(controlEffect>=0)
            {
                motor = controlEffect;
                direction = 1;
            }else if(controlEffect<0)
            {
                motor = ~controlEffect + 1;
                direction = 0;
            }else;
                        
            if(canRxMessage[0].data[4])
            {
                go = true;
                stop = false;
            }else
            {
                go = false;
                stop = true;
            }
            
            if(canRxMessage[0].data[5])
            {
                QEIPos = 0;
                QEIPosHigh = 0;
                POS1CNT = 0;
            }
            
		}
		if(canRxMessage[1].buffer_status==CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage[1]);			
			/* reset the flag when done */
			canRxMessage[1].buffer_status=CAN_BUF_EMPTY;
		}
        if(canRxMessage[2].buffer_status==CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage[2]);			
			/* reset the flag when done */
			canRxMessage[2].buffer_status=CAN_BUF_EMPTY;
		};
        
        if(stop){
        LATAbits.LATA7=1;
        }
        
        if(go){
        LATAbits.LATA7=0;
        }
        
        LATAbits.LATA10=direction;
        P2DC1=(5*motor/3);
    };
}