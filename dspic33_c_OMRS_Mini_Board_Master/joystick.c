/*
 * File:   joystick.c
 * Author: dracula
 *
 * Created on March 11, 2018, 5:42 PM
 */

#include "user.h"

float joystickGainKP = 0;
float joystickGainKD = 0;
float debounceTime = 0.0;
float eliminateJitter = 0.0;
bool debounce = false;
bool debounceEdge = NULL;

void sbus_decode(unsigned int *radioSignal,int *radioChannel)
{
    int channel = 15;
    unsigned int signal = 22;
    unsigned int shift = 3;
    for(channel=15;channel>=0;channel--)
    {
        if(shift<=8)
        {
            radioChannel[channel] = (radioSignal[signal]<<shift) + (radioSignal[signal-1]>>(8-shift));
            signal--;
            if(shift == 8)
            {
                signal--;
            };
        }else
        {
            radioChannel[channel] = (radioSignal[signal]<<shift) + (radioSignal[signal-1]<<(shift-8)) + (radioSignal[signal-2]>>(16-shift));
            signal -= 2;
        }        
        radioChannel[channel] = radioChannel[channel] & 0x7ff;
        shift += 3;
        if(shift>=11){shift=shift-8;}
    }
}

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

    if(radioChannel[4]==0x0620)
    {
//        controlEffect->x = matlabVoltage.command[0];
//        controlEffect->y = matlabVoltage.command[1];
//        controlEffect->z = matlabVoltage.command[2];
//        if(stop)
//        {
//            motor[0] = 0;
//            motor[1] = 0;
//            motor[2] = 0;
//        }
        controlEffect->x = motor[0]/100.0;
        controlEffect->y = motor[1]/100.0;
        controlEffect->z = motor[2]/100.0;
    }else if(radioChannel[4]==0x0400)
    {
        joystick->x = (radioChannel[0] - 0x0400)/672.0*10.0;

        joystick->y = (radioChannel[1] - 0x0400)/672.0*10.0;

        joystick->z = (0x0400 - radioChannel[3])/672.0*10.0;

        joystickGainKD = (radioChannel[5] - 0x0160)/1344.0*10.0;

        joystickGainKP = (radioChannel[6] - 0x0160)/1344.0*10.0;
        
//        ahrsAttitude->z = fmodf(ahrsAttitude->z,360);
//        ahrsAttitude->z = ahrsAttitude->z / 180 * PI;
        JCoeff->triMatrix[0][0] = cos(ahrsAttitude->z);
        JCoeff->triMatrix[0][1] = sin(ahrsAttitude->z);
        JCoeff->triMatrix[1][0] = -sin(ahrsAttitude->z);
        JCoeff->triMatrix[1][1] = cos(ahrsAttitude->z);
        JCoeff->triMatrix[2][2] = 1;
        
/* close loop */
        if(radioChannel[10]==0x0160)
        {
            v_equal(joystickError,v_minus(m_v_multiply(JBackMatrix,joystick),v_s_multiply(omega,P.r/P.n)));
        }else if(radioChannel[10]==0x0400)
        {
            v_equal(joystickError,v_minus(m_v_multiply(JBackMatrix,m_v_multiply(JCoeff,joystick)),v_s_multiply(omega,P.r/P.n)));
        }else
        {
            v_equal(joystickError,v_minus(joystick,v_s_multiply(omega,P.r/P.n)));
        }
//        v_equal(controlEffect,v_s_multiply(joystickError,joystickGainKP));
        v_equal(controlEffect,v_plus(v_s_multiply(joystickError,joystickGainKP),v_s_multiply(v_s_multiply(joystickError,1/delta),joystickGainKD)));
        
/* open loop */
//        v_equal(controlEffect,m_v_multiply(JBackMatrix,joystick));
        
//        controlEffect->x = joystick->x;
//        controlEffect->y = joystick->y;
//        controlEffect->z = joystick->z;
        if(radioChannel[8]==0x06A0)
        {
            go = 0;
            stop = 1;
        }else if(radioChannel[8]==0x0160)
        {
            go = 1;
            stop = 0;
        }else
        {
            go = 0;
            stop = 1;
        };
    }else
    {
        controlEffect = OMRS_controller(qd, dqd, ddqd, q, dq);
        if(radioChannel[8]==0x06A0)
        {
            go = 0;
            stop = 1;
        }else if(radioChannel[8]==0x0160)
        {
            go = 1;
            stop = 0;
        }else
        {
            go = 0;
            stop = 1;
        };
    }
    
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
    
    canTxMessage[0].data[0] =  ((long)(100*controlEffect->x))>>24;
    canTxMessage[0].data[1] =  ((long)(100*controlEffect->x))>>16;
    canTxMessage[0].data[2] =  ((long)(100*controlEffect->x))>>8;
    canTxMessage[0].data[3] =  ((long)(100*controlEffect->x));

    canTxMessage[1].data[0] =  ((long)(100*controlEffect->y))>>24;
    canTxMessage[1].data[1] =  ((long)(100*controlEffect->y))>>16;
    canTxMessage[1].data[2] =  ((long)(100*controlEffect->y))>>8;
    canTxMessage[1].data[3] =  ((long)(100*controlEffect->y));

    canTxMessage[2].data[0] =  ((long)(100*controlEffect->z))>>24;
    canTxMessage[2].data[1] =  ((long)(100*controlEffect->z))>>16;
    canTxMessage[2].data[2] =  ((long)(100*controlEffect->z))>>8;
    canTxMessage[2].data[3] =  ((long)(100*controlEffect->z));
}