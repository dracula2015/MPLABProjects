/*
 * File:   trajectory.c
 * Author: dracula
 *
 * Created on March 12, 2018, 5:45 PM
 */

#include "user.h"

float radius = 1;//0.3;
float speed = PI / 15;
float rectLength = 1.0;
float loopTime = 0.0;

void Trajectory(void)
{
    if(radioChannel[9]==0x0160)
    {
        qd->x = radius*cos(globalTime*speed);
        qd->y = radius*sin(globalTime*speed);
        qd->z = 0;
        dqd->x = -radius*speed*sin(globalTime*speed);
        dqd->y = radius*speed*cos(globalTime*speed);
        dqd->z = 0;
        ddqd->x = -radius*pow(speed,2)*cos(globalTime*speed);
        ddqd->y = -radius*pow(speed,2)*sin(globalTime*speed);
        ddqd->z = 0; 
    }else if(radioChannel[9]==0x0400)
    {
        qd->x = 2*radius*sin(globalTime*speed);
        qd->y = radius*sin(2*globalTime*speed);
        qd->z = 0;
        dqd->x = 2*radius*speed*cos(globalTime*speed);
        dqd->y = 2*radius*speed*cos(2*globalTime*speed);
        dqd->z = 0;
        ddqd->x = -2*radius*pow(speed,2)*sin(globalTime*speed);
        ddqd->y = -4*radius*pow(speed,2)*sin(globalTime*speed);
        ddqd->z = 0; 
    }else if(radioChannel[9]==0x06A0)
    {
        if(0<=fmodf(globalTime*speed,4*rectLength) && fmodf(globalTime*speed,4*rectLength)<rectLength)
        {
            qd->x = fmodf(globalTime*speed,4*rectLength);
            qd->y = 0;
        }
        else if(rectLength<=fmodf(globalTime*speed,4*rectLength) && fmodf(globalTime*speed,4*rectLength)<2*rectLength)
        {
            qd->x = rectLength;
            qd->y = fmodf(globalTime*speed,4*rectLength)-rectLength;
        }
        else if(2*rectLength<=fmodf(globalTime*speed,4*rectLength) && fmodf(globalTime*speed,4*rectLength)<3*rectLength)
        {
            qd->x = 3*rectLength-fmodf(globalTime*speed,4*rectLength);
            qd->y = rectLength;
        }
        else if(3*rectLength<=fmodf(globalTime*speed,4*rectLength) && fmodf(globalTime*speed,4*rectLength)<4*rectLength)
        {
            qd->x = 0;
            qd->y = 4*rectLength-fmodf(globalTime*speed,4*rectLength);
        }
        else;
        if (globalTime > 15)
        {
            qd->z = 0.35 * (globalTime - 15);
        }

        v_equal(dqd,v_s_multiply(v_minus(qd,qdPre),1/delta));
        v_equal(qdPre,qd);
        v_equal(ddqd,v_s_multiply(v_minus(dqd,dqdPre),1/delta));
        v_equal(dqdPre,dqd);
    }else;      
}
