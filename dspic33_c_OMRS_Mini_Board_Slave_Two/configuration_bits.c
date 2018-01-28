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

/******************************************************************************/
/* Configuration Bits                                                         */
/*                                                                            */
/* This is not all available configuration bits for all dsPIC devices.        */
/* Refer to the dsPIC device specific .h file in the compiler                 */
/* support\dsPIC33F\h directory for complete options specific to the device   */
/* selected.  For additional information about what hardware configurations   */
/* mean in terms of device operation, refer to the device datasheet           */
/* 'Special Features' chapter.                                                */
/*                                                                            */
/* A feature of MPLAB X is the 'Generate Source Code to Output' utility in    */
/* the Configuration Bits window.  Under Window > PIC Memory Views >          */
/* Configuration Bits, a user controllable configuration bits window is       */
/* available to Generate Configuration Bits source code which the user can    */
/* paste into this project.                                                   */
/******************************************************************************/

/* TODO Fill in your configuration bits from the config bits generator here.  */
/* TODO DSPIC33FJ128MC804 Configuration Bit Settings*/
// 'C' source line config statements
// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

/*
_FOSCSEL(FNOSC_FRC);                                  // Select Internal FRC at POR
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);       // Enable Clock Switching and Configure Posc in XT mode
_FPOR(RST_PWMPIN & PWM1H_ACT_HI & PWM1L_ACT_HI);      // High and Low switches set to active-high state 
*/

// FOSCSEL
#pragma config FNOSC = PRI              // Primary Oscillator (XT, HS, EV))
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = XT              // XT Oscillator Mode
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSECMD           // This bit is extremely important? if set to CSDCMD there will be no PWM signal!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//#pragma config FCKSM = CSDCMD         // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer always enabled)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON             // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD3               // Comm Channel Select (Communicate on PGC3/EMUC3 and PGD3/EMUD3)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)