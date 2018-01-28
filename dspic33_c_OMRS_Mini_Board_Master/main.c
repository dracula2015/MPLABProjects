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

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
/* i.e. uint16_t <variable_name>; */

bool go = 0;
bool stop = 0;
bool direction = 0;

int count[2]={0,0};
int motor = 0;
int i=0;
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
        
    canTxMessage[0].message_type=CAN_MSG_RTR;
    //canTxMessage.message_type=CAN_MSG_DATA;
    canTxMessage[0].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[0].buffer=0;
    canTxMessage[0].id=0x12344321;
    canTxMessage[0].data_length=4;
    
    canTxMessage[1].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[1].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[1].buffer=0;
    canTxMessage[1].id=0x12345667;
    canTxMessage[1].data[0]=0x55;
    canTxMessage[1].data[1]=0x55;
    canTxMessage[1].data[2]=0x55;
    canTxMessage[1].data_length=3;
    
    canTxMessage[2].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[2].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[2].buffer=1;
    canTxMessage[2].id=0x12345668;
    canTxMessage[2].data[0]=0x66;
    canTxMessage[2].data[1]=0x66;
    canTxMessage[2].data[2]=0x66;
    canTxMessage[2].data_length=3;
    
    canTxMessage[3].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[3].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[3].buffer=2;
    canTxMessage[3].id=0x12345669;
    canTxMessage[3].data[0]=0x77;
    canTxMessage[3].data[1]=0x77;
    canTxMessage[3].data[2]=0x77;
    canTxMessage[3].data_length=3;

    
    /* Delay for a second */
    Delay(Delay_1S_Cnt);

    /* send a CAN message */
    sendECAN(&canTxMessage[0]);
    sendECAN(&canTxMessage[1]);
    sendECAN(&canTxMessage[2]);
    sendECAN(&canTxMessage[3]);
    LATAbits.LATA8 = 1;
    LATCbits.LATC0 = 1;
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
        if(stop){
        LATAbits.LATA7=1;
        }
        
        if(go){
        LATAbits.LATA7=0;
        }
          
        if(i==0)
        {            
            motor = count[0];
            motor = motor & 0x00FF;
            motor = motor | (count[1]<<8);
        }
        
        {
            int temp=0;
            temp = motor & 0x8000;
            if(temp)
            {
                motor=motor & 0x7fff ;
                //motor[j]=~(motor[j]-1);
                direction=0;
            }else{direction=1;}
        }
        LATAbits.LATA10=direction;
        P2DC1=(5*motor/3);
        
        QEIPos = (QEIPosHigh << 16) + POS1CNT; 
//        U1TXREG = QEIPos >> 24;
//        U1TXREG = QEIPos >> 16;
//        U1TXREG = QEIPos >> 8;
//        U1TXREG = QEIPos;  

        /* check to see when a message is received and move the message 
		into RAM and parse the message */ 
		if(canRxMessage[0].buffer_status==CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage[0]);			
			/* reset the flag when done */
			canRxMessage[0].buffer_status=CAN_BUF_EMPTY;
//            U1TXREG = canRxMessage[0].data[0];
//            U1TXREG = canRxMessage[0].data[1];
		}
		else if(canRxMessage[1].buffer_status==CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage[1]);			
			/* reset the flag when done */
			canRxMessage[1].buffer_status=CAN_BUF_EMPTY;
//            U1TXREG = canRxMessage[1].data[0];
//            U1TXREG = canRxMessage[1].data[1];
		}
        else if(canRxMessage[2].buffer_status==CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage[2]);			
			/* reset the flag when done */
			canRxMessage[2].buffer_status=CAN_BUF_EMPTY;
//            U1TXREG = canRxMessage[2].data[0];
//            U1TXREG = canRxMessage[2].data[1];
		};
    };
}
