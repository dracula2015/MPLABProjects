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
    if(ReceivedChar == 'u')
    {
        U2TXREG = '#';
        U2TXREG = 'f';
    }
    IFS0bits.U1RXIF = 0;
}

void __attribute__((interrupt, auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // clear TX interrupt flag
}

void __attribute__((__interrupt__, auto_psv)) _U2RXInterrupt(void)
{
    ReceivedChar1 = U2RXREG;
    U1TXREG = ReceivedChar1;
    ahrs.signal[ahrsSignalCount] = ReceivedChar1;
    ahrsSignalCount += 1;
    if(ahrsSignalCount == 24)
    {
//        ahrs_decode(ahrs.signal,ahrs.attitude,ahrs.acclerom);
        U2TXREG = '#';
        U2TXREG = 'f';
        ahrsSignalCount = 0;
        ahrsMessage = true;
    }
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
	    /* check to see if buffer 1 is full */
	    if(C1RXFUL1bits.RXFUL1)
	    {			
			/* set the buffer full flag and the buffer received flag */
			canRxMessage[0].buffer_status=CAN_BUF_FULL;
			canRxMessage[0].buffer=1;	
		}		
		/* check to see if buffer 2 is full */
		if(C1RXFUL1bits.RXFUL2)
		{
			/* set the buffer full flag and the buffer received flag */
			canRxMessage[1].buffer_status=CAN_BUF_FULL;
			canRxMessage[1].buffer=2;					
		}
		/* check to see if buffer 3 is full */
		if(C1RXFUL1bits.RXFUL3)
		{
			/* set the buffer full flag and the buffer received flag */
			canRxMessage[2].buffer_status=CAN_BUF_FULL;
			canRxMessage[2].buffer=3;					
		}
//		else;
		/* clear flag */
		C1INTFbits.RBIF = 0;
	}
	if(C1INTFbits.TBIF)
    {
	    /* clear flag */
		C1INTFbits.TBIF = 0;	    
	}
	
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
    globalTime +=0.001;
//    LATAbits.LATA8 = ~LATAbits.LATA8;
//    LATCbits.LATC0 = ~LATCbits.LATC0;
    IFS1bits.T4IF = 0; // Clear Timer3 Interrupt Flag
}