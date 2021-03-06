/* 
 * File:   user.c   
 * Author: dracula
 * Comments:
 * Revision history: EDITION 0.1 
 */

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/
//#define MANUAL

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

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>
#include <p33FJ128MC804.h>         /* For true/false definition                     */
#include "user.h"            /* variables/params used by user.c               */

#define FCY 40000000
#define BAUDRATE1 115200//57600//9600
#define BAUDRATE2 100000//115200//57600//9600
#define BRGVAL1 ((FCY/BAUDRATE1)/16)-1
#define BRGVAL2 ((FCY/BAUDRATE2)/16)-1

#define FCAN  	      	40000000 
#define BITRATE 		1000000  
#define NTQ 			20		// 20 Time Quanta in a Bit Time
#define BRP_VAL			((FCAN/(2*NTQ*BITRATE))-1)

#define DELAY_105us asm volatile ("REPEAT,#4201");Nop();//105us delay 
#define DELAY_10us asm volatile ("REPEAT,#401");Nop();//10us delay 

float globalTime = 0.0;
float globalTimePre = 0.0;
float delta = 0.0;
unsigned char ReceivedChar, ReceivedChar1;
unsigned char TransmitChar, TransmitChar1;
long QEIPos = 0;
long QEIPosHigh = 0;
long wheelPos[3] = {0,0,0};
long wheelPosPre[3] = {0,0,0};
float wheelSpeed[3] = {0.0,0.0,0.0};
//float wheelSpeedPre[3] = {0.0,0.0,0.0};

bool go = 0;
bool stop = 1;
bool direction = 0;
bool reset = false;
float radioInterval = 0.0;
unsigned int radioSignalCount = 0;
//unsigned int ahrsSignalCount = 0;
unsigned int radioSignal[25] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int radioChannel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//unsigned char ahrsSignal[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};
//float ahrsAttitude[3] = {0.0, 0.0, 0.0};
//float ahrsAcclerom[3] = {0.0, 0.0, 0.0};

int count[6]={0,0,0,0,0,0};
int motor[3] = {0,0,0};
int hostCommandCount=0;

AHRS ahrs;
MATCOMMAND matlabVoltage;
QEISpeed qeiSpeed;
UART1MSGBUF uart1MsgBuf __attribute__((space(dma)));
bool QEIStatus;
/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

void InitApp(void)
{
    /* TODO Initialize User Ports/Peripherals/Project here */
    //*************************************************************
    // Unlock Registers
    //*************************************************************
    __builtin_write_OSCCONL(OSCCON & ~(1<<6)); 

    //*************************************************************
    // Configure Input Functions
    // (See Table 30-1)
    //*************************************************************
    //***************************
    // Assign U1Rx To Pin RP20
    //***************************
    /*square board*/
    RPINR18bits.U1RXR = 20;
    //***************************
    // Assign U2Rx To Pin RP17
    //***************************
    /*square board*/
    RPINR19bits.U2RXR = 21;
    //***************************
    // Assign U1CTS To Pin RP1
    //***************************
    //RPINR18bits.U1CTSR = 1;
 
    //*************************************************************
    // Configure Output Functions
    // (See Table 30-2)
    //*************************************************************
    //***************************
    // Assign U1Tx To Pin RP4
    //***************************
    /*square board*/
    RPOR2bits.RP4R = 3;
    //***************************
    // Assign U2Tx To Pin RP18
    //***************************
    /*square board*/
    RPOR9bits.RP18R = 5;
    //***************************
    // Assign U1RTS To Pin RP3
    //***************************
    //RPOR1bits.RP3R = 4;
    
    //*************************************************************
    //Assign QEI1 Phase A To Pin RP10
    //*************************************************************
    RPINR14bits.QEA1R = 10;
    
    //*************************************************************
    //Assign QEI1 Phase B To Pin RP11
    //*************************************************************
    RPINR14bits.QEB1R = 11;
    
    //*************************************************************
    //Assign QEI1 INDEX To Pin RP25
    //*************************************************************
    RPINR15bits.INDX1R = 25;
    
    //*************************************************************
    // Configure ECAN Module
    //*************************************************************
    //***************************
    // Assign ECAN1 C1RxD To Pin RP24
    //***************************
    RPINR26bits.C1RXR = 24;
    //***************************
    // Assign ECAN1 C1TxD To Pin RP14
    //***************************
    RPOR7bits.RP14R = 16;
    //*************************************************************
    // Lock Registers
    //*************************************************************
    __builtin_write_OSCCONL(OSCCON | (1<<6));

    /* Setup analog functionality and port direction */
    TRISAbits.TRISA7=0;
    TRISAbits.TRISA10=0;
    TRISAbits.TRISA8=0;
//    TRISCbits.TRISC0=0;//RP16
    TRISCbits.TRISC1=0;//RP17
//    TRISCbits.TRISC2=0;//RP28
//    TRISCbits.TRISC5=0;//RP21

    /*
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB5 = 1;
    */
    //TRISB = 0xFFFF;
    /* Initialize peripherals */
    cfgDma4UartTx();
    DMAInit();
    ECANInit();
    UartInit();
    QEInit();
    PwmInit();
    TimerInit();
}

void PwmInit(void)
{
        /*
         * PxTPER = FCY/(FPWM*PxTMR PreScaler)-1
         * PxTMR PreScaler=1:1
         * FPWM=20KHz
         * FCY=40MHz, refer to function ConfigureOscillator
         * PxTPER=1999
         * 0x07CF
         * 0b0000 0111 1100 1111
         */
        P2TCON=0x8000;   //or P1TCONbits.PTEN = 1;
        P2TMR=0x0000;
        P2TPER=1999;     // period:50us
        //P2SECMP=0x0000;
        PWM2CON1=0x0FFF;
        PWM2CON2=0x0004;
        P2OVDCON=0xFF00;
        
//        P1TCON=0x8000;   //or P1TCONbits.PTEN = 1;
//        P1TMR=0x0000;
//        P1TPER=1999;     // period:50us
//        //P1SECMP=0x0000;
//        PWM1CON1=0x0FFF;
//        PWM1CON2=0x0004;
//        P1OVDCON=0xFF00;
//        P1FLTACON=0x0080;
//        P1DTCON1=0x0000;
//        P1DTCON2=0x0000;
        // LSB is not used for duty cycle, the realy duty cycle count should be PIDC*/2 !!!!!!
}

void UartInit(void)
{
    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U1MODEbits.BRGH = 0; // Low Speed mode
    U1MODEbits.LPBACK = 0;
    U1BRG = BRGVAL1; // BAUD Rate Setting for 115200//57600
    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.URXISEL = 0;
    IEC0bits.U1TXIE = 1; // Enable UART Tx interrupt
    IEC0bits.U1RXIE = 1; // Enable UART Rx interrupt
    IPC2bits.U1RXIP = 0b100;
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx
    
    U2MODEbits.STSEL = 1; // 2-stop bit
    U2MODEbits.PDSEL = 1; // Even Parity, 8-data bits
    U2MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U2MODEbits.BRGH = 0; // Low Speed mode
    U2MODEbits.LPBACK = 0;
    U2BRG = BRGVAL2; // BAUD Rate Setting for 100000//115200//57600
    U2STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U2STAbits.UTXISEL1 = 0;
    U2STAbits.URXISEL = 0;
    IEC1bits.U2TXIE = 1; // Enable UART Tx interrupt
    IEC1bits.U2RXIE = 1; // Enable UART Rx interrupt
    IPC7bits.U2RXIP = 0b100;
    U2MODEbits.UARTEN = 1; // Enable UART
    U2STAbits.UTXEN = 1; // Enable UART Tx
    /* wait at least 104 usec (1/9600) before sending first char */
    //DELAY_105us
    /* wait at least 10 usec (1/115200) before sending first char */
    DELAY_10us 
}

void QEInit(void)
{
//    MAX1CNT = 36351; //512*71-1=36351
    MAX1CNT = 0xFFFF;
    IEC3bits.QEI1IE = 1;
    DFLT1CONbits.QEOUT = 1;
    DFLT1CONbits.QECK = 2;//1:4 digital filter clock devision
    QEI1CONbits.QEIM = 7;
    /*
    QEI1CONbits.QEIM = 6;
    QEI1CONbits.POSRES = 1;
    */
}

void TimerInit(void)
{
    /* This code generates an interrupt on every 1 second */
    T3CONbits.TON = 0; // Stop any 16-bit Timer3 operation
    T2CONbits.TON = 0; // Stop any 16/32-bit Timer2 operation
    T2CONbits.T32 = 1; // Enable 32-bit Timer mode
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR3 = 0x0000; // Clear 32-bit Timer (msw)
    TMR2 = 0x0000; // Clear 32-bit Timer (lsw)
    PR3 = 0x262; // Load 32-bit period value (msw)
    PR2 = 0x5A00; // Load 32-bit period value (lsw)
    IPC2bits.T3IP = 0x01; // Set Timer3 Interrupt Priority Level
    IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
    IEC0bits.T3IE = 1; // Enable Timer3 interrupt
    T2CONbits.TON = 1; // Start 32-bit Timer
    
    T4CONbits.TON = 0; // Stop any 16 Timer4 operation
    T4CONbits.TCS = 0; // Select internal instruction cycle clock
    T4CONbits.TGATE = 0; // Disable Gated Timer mode
    T4CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR4 = 0x0000; // Clear timer register
//    PR4 = 0x9C40; // Load the period value,40000*0.025us=1ms
    PR4 = 0xFA0; // Load the period value,4000*0.025us=0.1ms
    IPC6bits.T4IP = 0x01; // Set Timer4 Interrupt Priority Level
    IFS1bits.T4IF = 0; // Clear Timer4 Interrupt Flag
    IEC1bits.T4IE = 1; // Enable Timer4 interrupt
    T4CONbits.TON = 1; // Start Timer
}

// DMA4 configuration
void cfgDma4UartTx(void)
{
	//********************************************************************************
	//  Associate DMA Channel 4 with UART Tx
	//********************************************************************************/
    /* Data Transfer Size: Byte Transfer Mode */
    DMA4CONbits.SIZE = 0x1;
    /* Data Transfer Direction: DMA RAM to Peripheral */
    DMA0CONbits.DIR = 0x1;
    /* DMA Addressing Mode: Register Indirect with Post-Increment mode */
    DMA0CONbits.AMODE = 0x0;
    /* One-shot, Ping-Pong modes disabled */
    DMA4CONbits.MODE  = 0x1;

	DMA4REQ = 0x000C;					// Select UART1 Transmitter
//	DMA4PAD = (volatile unsigned int) &U1TXREG;
    DMA4PAD =  0x0224;
	
	//********************************************************************************
	//  Configure DMA Channel 4 to:
	//  Transfer data from RAM to UART
	//  One-Shot mode
	//  Register Indirect with Post-Increment
	//  Using single buffer
	//  13 transfers per buffer
	//  Transfer words
	//********************************************************************************/

	DMA4CNT = 13;						// 13 DMA requests

	//********************************************************************************
	// Associate one buffer with Channel 4 for one-shot operation
	//********************************************************************************/
	DMA4STA = __builtin_dmaoffset(&uart1MsgBuf[0]);

	//********************************************************************************
	//	Enable DMA Interrupts
	//********************************************************************************/
	IFS2bits.DMA4IF  = 0;			// Clear DMA Interrupt Flag
	IEC2bits.DMA4IE  = 1;			// Enable DMA interrupt

}

// DMA5 configuration
//void cfgDma5UartRx(void)
//{
//	//********************************************************************************
//	//  Associate DMA Channel 5 with UART Rx
//	//********************************************************************************/
//	/* Data Transfer Size: Word Transfer Mode */
//    DMA5CONbits.SIZE  = 0x0;
//    /* Data Transfer Direction: Peripheral to DMA RAM */
//    DMA5CONbits.DIR   = 0;
//    /* DMA Addressing Mode: Register Indirect with Post-Increment mode */
//    DMA5CONbits.AMODE = 0;
//    /*  Continuous, Ping-Pong modes enabled */
//	DMA5CONbits.MODE  = 2;
//
//    DMA5REQ = 0x000B;					// Select UART1 Receiver
////	DMA5PAD = (volatile unsigned int) &U1RXREG;
//    DMA5PAD = 0x0226;
//
//	//********************************************************************************
//	//  Configure DMA Channel 5 to:
//	//  Transfer data from UART to RAM Continuously
//	//  Register Indirect with Post-Increment
//	//  Using two ?ping-pong? buffers
//	//  8 transfers per buffer
//	//  Transfer words
//	//********************************************************************************/
//
//	DMA5CNT = 7;						// 8 DMA requests
//
//	//********************************************************************************
//	//  Associate two buffers with Channel 5 for ?Ping-Pong? operation
//	//********************************************************************************/
//	DMA5STA = __builtin_dmaoffset(&uart1MsgBuf[0]);
//	DMA5STB = __builtin_dmaoffset(&uart1MsgBuf[1]);
//
//	//********************************************************************************
//	//	Enable DMA Interrupts
//	//********************************************************************************/
//	IFS3bits.DMA5IF  = 0;			// Clear DMA interrupt
//	IEC3bits.DMA5IE  = 1;			// Enable DMA interrupt
//
//	//********************************************************************************
//	//  Enable DMA Channel 5 to receive UART data
//	//********************************************************************************/
//	DMA5CONbits.CHEN = 1;			// Enable DMA Channel
//}

