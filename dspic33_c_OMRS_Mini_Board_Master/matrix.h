/* 
 * File:   matrix.h   
 * Author: dracula
 * Comments:
 * Revision history: EDITION 1.0 
 */

// This is a guard condition so that contents of this file are not included more than once.  
#ifndef XC_HEADER_MATRIX_H
#define	XC_HEADER_MATRIX_H

// TODO Insert appropriate #include <>

#include "vector.h"

//object oriented c

typedef struct Matrix
{
    float triMatrix[3][3];

	int thisMatrixAddress;

	bool globalMatrix;
    
    //struct Matrix *(*m_constructor)(struct Matrix*m, float triMatrix[3][3], float x11, float x12, float x13, float x21, float x22, float x23, float x31, float x32, float x33);

    //void (*m_destructor)(struct Matrix*m, bool dynamic);

    //struct Matrix *(*m_plus)(struct Matrix*m,struct Matrix*n);

    //struct Matrix *(*m_minus)(struct Matrix*m,struct Matrix*n);

    //struct Matrix *(*m_m_multiply)(struct Matrix*m,struct Matrix*n);

    //Vector3f *(*m_v_multiply)(struct Matrix*m,Vector3f*v);
    //
    //struct Matrix *(*m_s_multiply)(struct Matrix *m,float s);
    //
    //float (*m_determinant)(struct Matrix *m);    
    //
    //struct Matrix *(*m_cofactor)(struct Matrix *m);    
    //
    //struct Matrix *(*m_inverse)(struct Matrix *m);
    //
    //struct Matrix *(*m_equal)(struct Matrix *m,struct Matrix *n);
}Matrix;

// TODO Insert declarations
Matrix *m_constructor(bool globalMatrix, Matrix*m, float triMatrix[3][3], float x11, float x12, float x13, float x21, float x22, float x23, float x31, float x32, float x33);

void m_destructor(Matrix*m, bool dynamic);

Matrix *m_plus(Matrix*m,Matrix*n);

Matrix *m_minus(Matrix*m,Matrix*n);

Matrix *m_m_multiply(Matrix*m,Matrix*n);

Vector3f *m_v_multiply(Matrix*m,Vector3f*v);

Matrix *m_s_multiply(Matrix *m,float s);

float m_determinant(Matrix *m);    

Matrix *m_cofactor(Matrix *m);    

Matrix *m_inverse(Matrix *m);

Matrix *m_equal(Matrix*m,Matrix*n);
// TODO Insert declarations

#endif

