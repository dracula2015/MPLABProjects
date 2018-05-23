/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__dsPIC33E__)
    	#include <p33Exxxx.h>
    #elif defined(__dsPIC33F__)
    	#include <p33Fxxxx.h>
    #endif
#endif

#include <stdint.h>        /* Includes uint16_t definition   */
#include <stdbool.h>       /* Includes true/false definition */
#include "user.h"

/******************************************************************************/
/* Interrupt Vector Options                                                   */
/******************************************************************************/
/*                                                                            */
/* Refer to the C30 (MPLAB C Compiler for PIC24F MCUs and dsPIC33F DSCs) User */
/* Guide for an up to date list of the available interrupt options.           */
/* Alternately these names can be pulled from the device linker scripts.      */
/*                                                                            */
/* dsPIC33F Primary Interrupt Vector Names:                                   */
/*                                                                            */
/* _INT0Interrupt      _C1Interrupt                                           */
/* _IC1Interrupt       _DMA3Interrupt                                         */
/* _OC1Interrupt       _IC3Interrupt                                          */
/* _T1Interrupt        _IC4Interrupt                                          */
/* _DMA0Interrupt      _IC5Interrupt                                          */
/* _IC2Interrupt       _IC6Interrupt                                          */
/* _OC2Interrupt       _OC5Interrupt                                          */
/* _T2Interrupt        _OC6Interrupt                                          */
/* _T3Interrupt        _OC7Interrupt                                          */
/* _SPI1ErrInterrupt   _OC8Interrupt                                          */
/* _SPI1Interrupt      _DMA4Interrupt                                         */
/* _U1RXInterrupt      _T6Interrupt                                           */
/* _U1TXInterrupt      _T7Interrupt                                           */
/* _ADC1Interrupt      _SI2C2Interrupt                                        */
/* _DMA1Interrupt      _MI2C2Interrupt                                        */
/* _SI2C1Interrupt     _T8Interrupt                                           */
/* _MI2C1Interrupt     _T9Interrupt                                           */
/* _CNInterrupt        _INT3Interrupt                                         */
/* _INT1Interrupt      _INT4Interrupt                                         */
/* _ADC2Interrupt      _C2RxRdyInterrupt                                      */
/* _DMA2Interrupt      _C2Interrupt                                           */
/* _OC3Interrupt       _DCIErrInterrupt                                       */
/* _OC4Interrupt       _DCIInterrupt                                          */
/* _T4Interrupt        _DMA5Interrupt                                         */
/* _T5Interrupt        _U1ErrInterrupt                                        */
/* _INT2Interrupt      _U2ErrInterrupt                                        */
/* _U2RXInterrupt      _DMA6Interrupt                                         */
/* _U2TXInterrupt      _DMA7Interrupt                                         */
/* _SPI2ErrInterrupt   _C1TxReqInterrupt                                      */
/* _SPI2Interrupt      _C2TxReqInterrupt                                      */
/* _C1RxRdyInterrupt                                                          */
/*                                                                            */
/* For alternate interrupt vector naming, simply add 'Alt' between the prim.  */
/* interrupt vector name '_' and the first character of the primary interrupt */
/* vector name.  There is no Alternate Vector or 'AIVT' for the 33E family.   */
/*                                                                            */
/* For example, the vector name _ADC2Interrupt becomes _AltADC2Interrupt in   */
/* the alternate vector table.                                                */
/*                                                                            */
/* Example Syntax:                                                            */
/*                                                                            */
/* void __attribute__((interrupt,auto_psv)) <Vector Name>(void)               */
/* {                                                                          */
/*     <Clear Interrupt Flag>                                                 */
/* }                                                                          */
/*                                                                            */
/* For more comprehensive interrupt examples refer to the C30 (MPLAB C        */
/* Compiler for PIC24 MCUs and dsPIC DSCs) User Guide in the                  */
/* <C30 compiler install directory>/doc directory for the latest compiler      */
/* release.  For XC16, refer to the MPLAB XC16 C Compiler User's Guide in the */
/* <XC16 compiler install directory>/doc folder.                               */
/*                                                                            */
/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* TODO Add interrupt routine code here. */
void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void)
{
    ReceivedChar = U1RXREG;
    U1TXREG = ReceivedChar;
    if(ReceivedChar == 'g'){go = 1; stop=0;}
    else 
    {
        if(ReceivedChar == 's'){stop = 1; go = 0;}
        else
        {
            if(ReceivedChar == 'u')
            { 
                U1TXREG = 'u'; 
                hostCommandCount = 0;
            }
            else
            {
//                matlabVoltage.signal[hostCommandCount] = ReceivedChar;
//                hostCommandCount++;
//                if(hostCommandCount>=12)
//                {
//                    hostCommandCount=0;
//                }

                count[hostCommandCount] = ReceivedChar;
                hostCommandCount++;
                if(hostCommandCount>=6) 
                {
                    hostCommandCount = 0;
                    motor[0] = count[0];
                    motor[0] = motor[0] & 0x00FF;
                    motor[0] = motor[0] | (count[1]<<8);

                    motor[1] = count[2];
                    motor[1] = motor[1] & 0x00FF;
                    motor[1] = motor[1] | (count[3]<<8);

                    motor[2] = count[4];
                    motor[2] = motor[2] & 0x00FF;
                    motor[2] = motor[2] | (count[5]<<8);

                    int j=0;
                    for(j=0;j<3;j++)
                    {
                        int temp = 0;
                        temp = motor[j] & 0x8000;
                        if(temp)
                        {
                            motor[j] = motor[j] & 0x7fff ;
                            motor[j] = -motor[j];
                        }else;
                    }
                }
            }
        }
    }
//    sendECAN(&canTxMessage[3]);
    IFS0bits.U1RXIF = 0;
}

void __attribute__((interrupt, auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // clear TX interrupt flag
}

void __attribute__((__interrupt__, auto_psv)) _U2RXInterrupt(void)
{
    ReceivedChar1 = U2RXREG;
//    U1TXREG = ReceivedChar1;
    if(radioInterval>0.01)
    {    
        radioSignalCount = 0;
    }
    radioSignal[radioSignalCount] = ReceivedChar1;
    radioSignalCount += 1;
    if(radioSignalCount == 25)
    {
        if(radioSignal[0] == 0x0f)
        {
            sbus_decode(radioSignal,radioChannel);
        }
        radioSignalCount = 0;
    }
    radioInterval = 0.0;
    IFS1bits.U2RXIF = 0;
}

void __attribute__((interrupt, auto_psv)) _U2TXInterrupt(void)
{
    IFS1bits.U2TXIF = 0; // clear TX interrupt flag
}

void __attribute__((interrupt, auto_psv)) _QEI1Interrupt(void)
{    
    if(QEI1CONbits.UPDN == 1){QEIPosHigh += 1;}
    else {QEIPosHigh -= 1;}
    IFS3bits.QEI1IF = 0;
}

void __attribute__((interrupt,no_auto_psv))_C1Interrupt(void)  
{
	/* check to see if the interrupt is caused by receive */     	 
    if(C1INTFbits.RBIF)
    {
	    /* check to see if buffer 3 is full */
	    if(C1RXFUL1bits.RXFUL3)
	    {			
			/* set the buffer full flag and the buffer received flag */
			canRxMessage[0].buffer_status=CAN_BUF_FULL;
			canRxMessage[0].buffer=3;	
//            U1TXREG = 0x01;
		}		
		/* check to see if buffer 4 is full */
		if(C1RXFUL1bits.RXFUL4)
		{
			/* set the buffer full flag and the buffer received flag */
			canRxMessage[1].buffer_status=CAN_BUF_FULL;
			canRxMessage[1].buffer=4;
//            U1TXREG = 0x02;
		}
		/* check to see if buffer 5 is full */
		if(C1RXFUL1bits.RXFUL5)
		{
			/* set the buffer full flag and the buffer received flag */
			canRxMessage[2].buffer_status=CAN_BUF_FULL;
			canRxMessage[2].buffer=5;
//            U1TXREG = 0x03;
		}
        /* check to see if buffer 8 is full */
	    if(C1RXFUL1bits.RXFUL8)
	    {			
			/* set the buffer full flag and the buffer received flag */
			canRxMessage[3].buffer_status=CAN_BUF_FULL;
			canRxMessage[3].buffer=8;	
//            U1TXREG = 0x04;
		}		
		/* check to see if buffer 9 is full */
		if(C1RXFUL1bits.RXFUL9)
		{
			/* set the buffer full flag and the buffer received flag */
			canRxMessage[4].buffer_status=CAN_BUF_FULL;
			canRxMessage[4].buffer=9;
//            U1TXREG = 0x05;
		}
		/* check to see if buffer 10 is full */
		if(C1RXFUL1bits.RXFUL10)
		{
			/* set the buffer full flag and the buffer received flag */
			canRxMessage[5].buffer_status=CAN_BUF_FULL;
			canRxMessage[5].buffer=10;
//            U1TXREG = 0x06;
		}
		/* clear flag */
		C1INTFbits.RBIF = 0;
//        U1TXREG = 0x00;
	}
	else if(C1INTFbits.TBIF)
    {
	    /* clear flag */
		C1INTFbits.TBIF = 0;
//        U1TXREG = 0x07;	    
	}
	else;
//	U1TXREG = 0x08;
	/* clear interrupt flag */
	IFS2bits.C1IF=0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)
{
    /* Interrupt Service Routine code goes here */
//    LATCbits.LATC0 = ~LATCbits.LATC0;
//    LATAbits.LATA8 = ~LATAbits.LATA8;
    IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void)
{
    /* Interrupt Service Routine code goes here */
    globalTime +=0.0001;
    radioInterval += 0.0001;
//    LATAbits.LATA8 = ~LATAbits.LATA8;
//    LATCbits.LATC0 = ~LATCbits.LATC0;
    IFS1bits.T4IF = 0; // Clear Timer3 Interrupt Flag
}

//********************************************************************************
//	Setup DMA interrupt handlers
//********************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA4Interrupt(void)
{
    DMA4CONbits.CHEN  = 1;			// Re-enable DMA0 Channel
    DMA4REQbits.FORCE = 1;			// Manual mode: Kick-start the first transfer
	IFS2bits.DMA4IF = 0;			// Clear the DMA4 Interrupt Flag;
}

//void __attribute__((interrupt, no_auto_psv)) _DMA5Interrupt(void)
//{
//	static unsigned int BufferCount = 0;  // Keep record of which buffer contains Rx Data
//
//	if(BufferCount == 0)
//	{
////		DMA4STA = __builtin_dmaoffset(UART1MSGBUF[0]); // Point DMA 4 to data to be transmitted
//	}
//	else
//	{
////		DMA4STA = __builtin_dmaoffset(UART1MSGBUF[1]); // Point DMA 4 to data to be transmitted
//	}
//
//	DMA4CONbits.CHEN  = 1;			// Re-enable DMA4 Channel
//	DMA4REQbits.FORCE = 1;			// Manual mode: Kick-start the first transfer
//
//	BufferCount ^= 1;				
//	IFS3bits.DMA5IF = 0;			// Clear the DMA5 Interrupt Flag
//}