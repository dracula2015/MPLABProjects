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
bool stop = 0;
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

    /* configure and send a message */
    canTxMessage.message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage.frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage.buffer=0;
    canTxMessage.id=0x12345679;
    canTxMessage.data_length=4;

    QEIPos = (QEIPosHigh << 16) + POS1CNT; 
    canTxMessage.data[0] = QEIPos >> 24;
    canTxMessage.data[1] = QEIPos >> 16;
    canTxMessage.data[2] = QEIPos >> 8;
    canTxMessage.data[3] = QEIPos;
    ecanRtrRespond(&canTxMessage);

    /* send a CAN message */
    sendECAN(&canTxMessage);
    
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
        canTxMessage.data[0] = QEIPos >> 24;
        canTxMessage.data[1] = QEIPos >> 16;
        canTxMessage.data[2] = QEIPos >> 8;
        canTxMessage.data[3] = QEIPos;
        ecanRtrRespond(&canTxMessage);

        /* check to see when a message is received and move the message 
		into RAM and parse the message */ 
		if(canRxMessage[0].buffer_status==CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage[0]);			
			/* reset the flag when done */
			canRxMessage[0].buffer_status=CAN_BUF_EMPTY;
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
            controlEffect = canRxMessage[2].data[0];
            controlEffect = (controlEffect<<8) + canRxMessage[2].data[1];
            controlEffect = (controlEffect<<8) + canRxMessage[2].data[2];
            controlEffect = (controlEffect<<8) + canRxMessage[2].data[3];
            
            if(controlEffect>=0)
            {
                motor = controlEffect;
                direction = 1;
            }else if(controlEffect<0)
            {
                motor = ~controlEffect + 1;
                direction = 0;
            }else;
                        
            if(canRxMessage[1].data[4])
            {
                go = true;
                stop = false;
            }else
            {
                go = false;
                stop = true;
            }
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