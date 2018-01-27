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
#define BAUDRATE 115200//57600//9600  
#define BRGVAL ((FCY/BAUDRATE)/16)-1

#define FCAN  	      	40000000 
#define BITRATE 		1000000  
#define NTQ 			20		// 20 Time Quanta in a Bit Time
#define BRP_VAL			((FCAN/(2*NTQ*BITRATE))-1)

#define DELAY_105us asm volatile ("REPEAT,#4201");Nop();//105us delay 
#define DELAY_10us asm volatile ("REPEAT,#401");Nop();//10us delay 

float globalTime = 0;
long gloalTimeCount = 0;
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
    TRISCbits.TRISC0=0;
    /*
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB5 = 1;
    */
    //TRISB = 0xFFFF;
    /* Initialize peripherals */
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
    U1BRG = BRGVAL; // BAUD Rate Setting for 57600
    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.URXISEL = 0;
    IEC0bits.U1TXIE = 1; // Enable UART Tx interrupt
    IEC0bits.U1RXIE = 1; // Enable UART Rx interrupt
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx
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
    /* This code generates an interrupt on every second */
    ///*
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
    //*/
    
    T4CONbits.TON = 0; // Stop any 16 Timer4 operation
    T4CONbits.TCS = 0; // Select internal instruction cycle clock
    T4CONbits.TGATE = 0; // Disable Gated Timer mode
    T4CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR4 = 0x0000; // Clear timer register
    PR4 = 0x9C40; // Load the period value,40000*0.025us=1ms
    IPC6bits.T4IP = 0x01; // Set Timer4 Interrupt Priority Level
    IFS1bits.T4IF = 0; // Clear Timer4 Interrupt Flag
    IEC1bits.T4IE = 1; // Enable Timer4 interrupt
    T4CONbits.TON = 1; // Start Timer
}