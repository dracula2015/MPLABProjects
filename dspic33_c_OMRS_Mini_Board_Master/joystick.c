/*
 * File:   joystick.c
 * Author: dracula
 *
 * Created on March 11, 2018, 5:42 PM
 */

#include "user.h"

float joystickGainKP = 0;
float joystickGainKI = 0;
float debounceTime = 0.0;
float eliminateJitter = 0.0;
bool debounce = false;
bool debounceEdge = NULL;

void Debounce(void)
{
    if(debounce==false)
    {
        if(radioChannel[7]==0x06A0)
        {
            debounce = true;
            debounceEdge = true;
            LATCbits.LATC1 = 1;
        }
    }
    if(debounce==true)
    {
        if(radioChannel[7]==0x0160)
        {
            debounce = false;
            debounceEdge = false;
            debounceTime = eliminateJitter;
            eliminateJitter = 0.0;
            if(debounceTime>0.1)
            {
//                U1TXREG = ((long)(debounceTime*10000))>>8;
//                U1TXREG = (debounceTime*10000);
//                go = !go;
//                stop = !stop;
                reset = true;
                debounceTime = 0;
            }
            LATCbits.LATC1 = 0;
        }
    }
    if(debounceEdge==true)
    {
        eliminateJitter = eliminateJitter + delta;
    }
}

void Joystick(void)
{
    Debounce();
    if(radioChannel[8]==0x06A0)
    {
        go = 0;
        stop = 1;
    }else if(radioChannel[8]==0x0160)
    {
        go = 1;
        stop = 0;
    }else;
        
    if(stop){
        canTxMessage[0].data[4] = 0;
        canTxMessage[1].data[4] = 0;
        canTxMessage[2].data[4] = 0;
    }

    if(go){
        canTxMessage[0].data[4] = 1;
        canTxMessage[1].data[4] = 1;
        canTxMessage[2].data[4] = 1;
    }

    if(radioChannel[4]==0x0620)
    {
        controlEffect->x = motor[0];
        controlEffect->y = motor[1];
        controlEffect->z = motor[2];            
    }else if(radioChannel[4]==0x0400)
    {
        joystick->x = (radioChannel[0] - 0x0400)/672.0*10.0;

        joystick->y = (radioChannel[1] - 0x0400)/672.0*10.0;

        joystick->z = (0x0400 - radioChannel[3])/672.0*10.0;

        joystickGainKI = (radioChannel[5] - 0x0160)/1344.0*100.0;

        joystickGainKP = (radioChannel[6] - 0x0160)/1344.0*100.0;
/* close loop */
        v_equal(joystickError,v_minus(m_v_multiply(JBackMatrix,joystick),v_s_multiply(omega,P.r/P.n)));
        v_equal(joystickIntegral,v_plus(joystickIntegralPre,v_s_multiply(joystickError,delta)));
        v_equal(joystickIntegralPre,joystickIntegral);
        v_equal(joystickControl,v_plus(v_s_multiply(joystickError,joystickGainKP),v_s_multiply(joystickIntegral,joystickGainKI)));
        v_equal(controlEffect,joystickControl);
             
//        v_equal(joystickError,v_minus(joystick,omega));
//        v_equal(joystickControl,v_s_multiply(joystickError,joystickGainKP));
//        v_equal(controlEffect,joystickControl);
        
/* open loop */
//        v_equal(controlEffect,m_v_multiply(JBackMatrix,joystick));
        
//        controlEffect->x = joystick->x;
//        controlEffect->y = joystick->y;
//        controlEffect->z = joystick->z;
    }else;
}