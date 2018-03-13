/*
 * File:   parameters.c
 * Author: dracula
 *
 * Created on March 12, 2018, 3:36 PM
 */

#include "user.h"

Matrix *Jacobin;
Matrix *JConst;
Matrix *JCoeff;
Matrix* JBackMatrix;
Vector3f* qd;
Vector3f* dqd;
Vector3f* ddqd;
Vector3f* qdPre;
Vector3f* dqdPre;
Vector3f* q;
Vector3f* qPre;
Vector3f* dq;
Vector3f* omega;
Vector3f* controlEffect;
Vector3f* joystick;
Vector3f* joystickError;
Vector3f* joystickIntegral;
Vector3f* joystickIntegralPre;
Vector3f* joystickControl;

void InitialParameters(void)
{
    P.m = 11.4;
    P.Iv = 0.65;
    P.r = 0.05;
    P.Din = 0.147;
    P.Dout = 0.236;
    //P.La = (P.Din + P.Dout) / 2;
    P.La = 0.2425;
    P.I0 = 6 * pow(10, -6);
    P.kt = 0.0208;
    //P.kb = 1 / 34.34;
    P.kb = 0.02076;
    P.n = 71;
    P.b0 = 6.0 * pow(10, -5);
    P.Ra = 1.53;
    P.beta0 = pow(P.n, 2) * P.I0 / pow(P.r, 2);
    P.beta1 = pow(P.n, 2) * (P.b0 + P.kt*P.kb / P.Ra) / pow(P.r, 2);
    P.beta2 = P.n*P.kt / P.r / P.Ra;
    
    Jacobin = m_constructor(global, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    JCoeff = m_constructor(global, NULL, NULL, 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Kp = m_constructor(global, NULL, NULL, 6, 0, 0, 0, 6, 0, 0, 0, 6);
	Kd = m_constructor(global, NULL, NULL, 10, 0, 0, 0, 10, 0, 0, 0, 10);
    qd = v_constructor(global, NULL, 0, 0, 0);
	dqd = v_constructor(global, NULL, 0, 0, 0);
	ddqd = v_constructor(global, NULL, 0, 0, 0);
    qdPre = v_constructor(global, NULL, 0, 0, 0);
	dqdPre = v_constructor(global, NULL, 0, 0, 0);
    q = v_constructor(global, NULL, 0, 0, 0);
    qPre = v_constructor(global, NULL, 0, 0, 0);
	dq = v_constructor(global, NULL, 0, 0, 0);
    omega = v_constructor(global, NULL, 0, 0, 0);
    joystick = v_constructor(global, NULL, 0, 0, 0);
    joystickError = v_constructor(global, NULL, 0, 0, 0);
    joystickIntegral = v_constructor(global, NULL, 0, 0, 0);
    joystickIntegralPre = v_constructor(global, NULL, 0, 0, 0);
    joystickControl = v_constructor(global, NULL, 0, 0, 0);
    
    JBackMatrix = m_constructor(global, NULL, NULL, -0.5, sqrt(3)/2, P.La, -0.5, -sqrt(3)/2, P.La, 1, 0, P.La);
    JConst = m_constructor(global, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    m_equal(JConst,m_inverse(JBackMatrix));
    JCoeff->triMatrix[2][2] = P.r/P.n;
    
    /* robot was default controlled by the MCU*/
    radioSignal[4] = 0x01E0;
//    float Jcoefficient=0.0;
    canTxMessage[0].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[0].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[0].buffer=0;
    canTxMessage[0].id=0x12345667;
    canTxMessage[0].data[0]=0x00;
    canTxMessage[0].data[1]=0x00;
    canTxMessage[0].data[2]=0x00;
    canTxMessage[0].data[3]=0x00;
    canTxMessage[0].data[4]=0x00;
    canTxMessage[0].data[5]=0x00;
    canTxMessage[0].data[6]=0x00;
    canTxMessage[0].data[7]=0x00;
    canTxMessage[0].data_length=8;
    
    canTxMessage[1].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[1].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[1].buffer=1;
    canTxMessage[1].id=0x12345668;
    canTxMessage[1].data[0]=0x00;
    canTxMessage[1].data[1]=0x00;
    canTxMessage[1].data[2]=0x00;
    canTxMessage[1].data[3]=0x00;
    canTxMessage[1].data[4]=0x00;
    canTxMessage[1].data[5]=0x00;
    canTxMessage[1].data[6]=0x00;
    canTxMessage[1].data[7]=0x00;
    canTxMessage[1].data_length=8;
    
    canTxMessage[2].message_type=CAN_MSG_DATA;
    //canTxMessage.message_type=CAN_MSG_RTR;
    canTxMessage[2].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[2].buffer=2;
    canTxMessage[2].id=0x12345669;
    canTxMessage[2].data[0]=0x00;
    canTxMessage[2].data[1]=0x00;
    canTxMessage[2].data[2]=0x00;
    canTxMessage[2].data[3]=0x00;
    canTxMessage[2].data[4]=0x00;
    canTxMessage[2].data[5]=0x00;
    canTxMessage[2].data[6]=0x00;
    canTxMessage[2].data[7]=0x00;
    canTxMessage[2].data_length=8;
            
    canTxMessage[3].message_type=CAN_MSG_RTR;
    //canTxMessage.message_type=CAN_MSG_DATA;
    canTxMessage[3].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[3].buffer=6;
    canTxMessage[3].id=0x12344321;
    canTxMessage[3].data_length=4;
    
    //canTxMessage[4].message_type=CAN_MSG_RTR;
    canTxMessage[4].message_type=CAN_MSG_DATA;
    canTxMessage[4].frame_type=CAN_FRAME_EXT;
    //canTxMessage.frame_type=CAN_FRAME_STD;
    canTxMessage[4].buffer=7;
    canTxMessage[4].id=0x12312321;
    canTxMessage[4].data[0]=0x00;
    canTxMessage[4].data[1]=0x00;
    canTxMessage[4].data[2]=0x00;
    canTxMessage[4].data[3]=0x00;
    canTxMessage[4].data[4]=0x00;
    canTxMessage[4].data[5]=0x00;
    canTxMessage[4].data[6]=0x00;
    canTxMessage[4].data[7]=0x00;
    canTxMessage[4].data_length=8;
}