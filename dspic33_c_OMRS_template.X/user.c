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

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */

#include "user.h"            /* variables/params used by user.c               */

#define FCY 40000000
#define BAUDRATE 57600//9600  
#define BRGVAL ((FCY/BAUDRATE)/16)-1
#define DELAY_105us ams volatile ("REPEAT,#4201");Nop();//105us delay 
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
    //*******************************
    // Assign U1Rx To Pin RP0
    //***************************
    //RPINR18bits.U1RXR = 0;
    RPINR18bits.U1RXR = 4;
    
    //***************************
    // Assign U1CTS To Pin RP1
    //***************************
    //RPINR18bits.U1CTSR = 1;
    
    //*************************************************************
    // Configure Output Functions
    // (See Table 30-2)
    //*************************************************************
    //***************************
    // Assign U1Tx To Pin RP2
    //***************************
    //RPOR1bits.RP2R = 3;
    RPOR0bits.RP0R = 3;
    
    //***************************
    // Assign U1RTS To Pin RP3
    //***************************
    //RPOR1bits.RP3R = 4;
    
    //*************************************************************
    // Lock Registers
    //*************************************************************
    __builtin_write_OSCCONL(OSCCON | (1<<6));

    /* Setup analog functionality and port direction */
    TRISAbits.TRISA0=0;
    TRISAbits.TRISA1=0;
    
    TRISAbits.TRISA4=0;
    TRISAbits.TRISA7=0;
    
    TRISAbits.TRISA8=0;
    TRISAbits.TRISA9=0;
    /*
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB5 = 1;
    */
    //TRISB = 0xFFFF;
    /* Initialize peripherals */
    PwmInit();
    UartInit();
}

void PwmInit(void)
{
#if 1
    /*
     * PxTPER = FCY/(FPWM*PxTMR PreScaler)-1
     * PxTMR PreScaler=1:1
     * FPWM=20KHz
     * FCY=40MHz, refer to function ConfigureOscillator
     * PxTPER=1999
     * 0x07CF
     * 0b0000 0111 1100 1111
     */
    P1TCON=0x8000;   //or P1TCONbits.PTEN = 1;
    P1TMR=0x0000;
    P1TPER=1999;     // period:50us
    //P1SECMP=0x0000;
    PWM1CON1=0x0FFF;
    PWM1CON2=0x0004;
    P1OVDCON=0xFF00;
    //P1FLTACON=0x0080;
    //P1DTCON1=0x0000;
    //P1DTCON2=0x0000;
    P1DC1=100;
    P1DC2=200;
    P1DC3=1000;
    // LSB is not used for duty cycle, the realy duty cycle count should be PIDC*/2 !!!!!!
#endif
    
#if 0
/* Configuration register FPOR */

/* PWM time base operates in a Free Running mode */
P1TCONbits.PTMOD = 0b00;

/* PWM time base input clock period is TCY (1:1 prescale) */
/* PWM time base output post scale is 1:1 */
P1TCONbits.PTCKPS = 0b00;
P1TCONbits.PTOPS = 0b00;

/* Choose PWM time period based on input clock selected */
/* Refer to Equation: PxTPER=FCY/(FPWM × PxTMR PreScaler)-1*/
/* PWM switching frequency is 20 kHz */
/* FCY is 40 MHz */
P1TPER = 1999;

/* PWM I/O pairs 1 to 3 are in complementary mode */
/* PWM pins are enabled for PWM output */
PWM1CON1bits.PMOD1 = 0;
PWM1CON1bits.PMOD2 = 0;
PWM1CON1bits.PMOD3 = 0;
PWM1CON1bits.PEN1H = 1;
PWM1CON1bits.PEN2H = 1;
PWM1CON1bits.PEN3H = 1;
PWM1CON1bits.PEN1L = 1;
PWM1CON1bits.PEN2L = 1;
PWM1CON1bits.PEN3L = 1;
/* Immediate update of PWM enabled */
PWM1CON2bits.IUE = 1;

/* PWM I/O pin controlled by PWM Generator */
P1OVDCONbits.POVD3H = 1;
P1OVDCONbits.POVD2H = 1;
P1OVDCONbits.POVD1H = 1;
P1OVDCONbits.POVD3L = 1;
P1OVDCONbits.POVD2L = 1;
P1OVDCONbits.POVD1L = 1;

/* Initialize duty cycle values for PWM1, PWM2 and PWM3 signals */
P1DC1 = 200;
P1DC2 = 200;
P1DC3 = 200;
P1TCONbits.PTEN = 1;
#endif
}

void UartInit(void)
{
    int i;
    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U1MODEbits.BRGH = 0; // Low Speed mode
    U1MODEbits.LPBACK = 0;
    U1BRG = BRGVAL; // BAUD Rate Setting for 9600
    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.URXISEL = 0;
    IEC0bits.U1TXIE = 1; // Enable UART Tx interrupt
    IEC0bits.U1RXIE = 1; // Enable UART Rx interrupt
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx
    /* wait at least 104 usec (1/9600) before sending first char */
    //DELAY_105us
    for(i = 0; i < 4160; i++)
    {
    Nop();
    }

}
