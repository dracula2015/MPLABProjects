/* 
 * File:   vector.h
 * Author: dracula
 * Comments:
 * Revision history: EDITION 1.0
 */

// This is a guard condition so that contents of this file are not included more than once.  
#ifndef XC_HEADER_VECTOR_H
#define	XC_HEADER_VECTOR_H

#ifdef _WIN32
    #define printDetail
    #define printTimeConsumption
#endif

#define DELAY_105us asm volatile ("REPEAT,#4201");Nop();//105us delay 
#define DELAY_10us asm volatile ("REPEAT,#401");Nop();//10us delay 
#define global true
#define local false
// TODO Insert appropriate #include <>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>        /* Includes int definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <math.h>
#include <time.h>
//object oriented c

typedef struct Vector3f
{
    float x,y,z;

	int thisVectorAddress;

	bool globalVector;
    
    //struct Vector3f *(*v_constructor)(struct Vector3f*v, float x, float y, float z);

    //void (*v_destructor)(struct Vector3f*v, bool dynamic);

    //struct Vector3f *(*v_plus)(struct Vector3f*v,struct Vector3f*w);

    //struct Vector3f *(*v_minus)(struct Vector3f*v,struct Vector3f*w);

    //float (*v_v_multiply)(struct Vector3f*v,struct Vector3f*w);

    //struct Vector3f *(*v_s_multiply)(struct Vector3f*v,float s);

    //struct Vector3f *(*v_equal)(struct Vector3f*v,struct Vector3f*w);

    //float (*v_length)(struct Vector3f*v);

    //void (*v_normalize)(struct Vector3f*v);
}Vector3f;

// TODO Insert declarations
Vector3f *v_constructor(bool globalVector, Vector3f*v, float x, float y, float z);

void v_destructor(Vector3f*v, bool dynamic);

Vector3f *v_plus(Vector3f*v,Vector3f*w);

Vector3f *v_minus(Vector3f*v,Vector3f*w);

float v_v_multiply(Vector3f*v,Vector3f*w);

Vector3f *v_s_multiply(Vector3f*v,float s);

Vector3f *v_equal(Vector3f*v,Vector3f*w);

float v_length(Vector3f*v);

void v_normalize(Vector3f*v);
// TODO Insert declarations

void freeLocalMem(void);

void freeGlobalMem(void);

#endif
