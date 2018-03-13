/*
 * File:   matrix.c
 * Author: dracula
 *
 * Created on December 28, 2016, 10:06 PM
 */

#include "user.h"
extern int countMatrix;
extern int countMatrixGlobal;
extern Matrix *pointerMatrix[];
extern Matrix *pointerMatrixGlobal[];
//Matrix *m_constructor(Matrix*m, float *triMatrix[3][3], float x11=0, float x12=0, float x13=0, float x21=0, float x22=0, float x23=0, float x31=0, float x32=0, float x33=0)
Matrix *m_constructor(bool globalMatrix, Matrix*m, float triMatrix[3][3], float x11, float x12, float x13, float x21, float x22, float x23, float x31, float x32, float x33)
{
    if(m == NULL)
    {
		if (!globalMatrix)
		{
			pointerMatrix[countMatrix] = (Matrix*)malloc(sizeof(Matrix));
			m = pointerMatrix[countMatrix];
			m->thisMatrixAddress = countMatrix;
			countMatrix++;
		}
		else {
			pointerMatrixGlobal[countMatrixGlobal] = (Matrix*)malloc(sizeof(Matrix));
			m = pointerMatrixGlobal[countMatrixGlobal];
			m->thisMatrixAddress = countMatrixGlobal;
			countMatrixGlobal++;
		}
	}

	m->globalMatrix = globalMatrix;
    int i,j;
    if(triMatrix == NULL)
    {
        m->triMatrix[0][0] = x11;
        m->triMatrix[0][1] = x12;
        m->triMatrix[0][2] = x13;
        m->triMatrix[1][0] = x21;
        m->triMatrix[1][1] = x22;
        m->triMatrix[1][2] = x23;
        m->triMatrix[2][0] = x31;
        m->triMatrix[2][1] = x32;
        m->triMatrix[2][2] = x33;
    }
    else
    {
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            m->triMatrix[i][j] = triMatrix[i][j];
        }
    };
    }
    //m->m_plus = m_plus;
    //m->m_minus = m_minus;
    //m->m_m_multiply = m_m_multiply;
    //m->m_v_multiply = m_v_multiply;
    //m->m_s_multiply = m_s_multiply;
    //m->m_determinant = m_determinant;
    //m->m_cofactor = m_cofactor;
    //m->m_inverse = m_inverse;
    //m->m_equal = m_equal;
    return m;
};

void m_destructor(Matrix*m, bool dynamic)
{
    if(dynamic)
    {
		if (m->globalMatrix == false)
		{
			free(m);
		}
    }
};

Matrix *m_plus(Matrix*m,Matrix*n)
{
    int i,j;
    Matrix *temp = m_constructor(local, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            temp->triMatrix[i][j] = m->triMatrix[i][j] + n->triMatrix[i][j];
        }
    }
    return temp;
};

Matrix *m_minus(Matrix*m,Matrix*n)
{    
    int i,j;
    Matrix *temp = m_constructor(local, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            temp->triMatrix[i][j] = m->triMatrix[i][j] - n->triMatrix[i][j];
        }
    }
    return temp;
};

Matrix *m_m_multiply(Matrix*m,Matrix*n)
{

    int i,j;
    Matrix *temp = m_constructor(local, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            temp->triMatrix[i][j] = m->triMatrix[i][0] * n->triMatrix[0][j] + m->triMatrix[i][1] * n->triMatrix[1][j] +m->triMatrix[i][2] * n->triMatrix[2][j];
        }
    }
    return temp;
};

Vector3f *m_v_multiply(Matrix*m,Vector3f *v)
{
    Vector3f *temp = v_constructor(local, NULL, 0, 0, 0);
    temp->x = m->triMatrix[0][0] * v->x + m->triMatrix[0][1] * v->y + m->triMatrix[0][2] * v->z;
    temp->y = m->triMatrix[1][0] * v->x + m->triMatrix[1][1] * v->y + m->triMatrix[1][2] * v->z;
    temp->z = m->triMatrix[2][0] * v->x + m->triMatrix[2][1] * v->y + m->triMatrix[2][2] * v->z;
    return temp;
};

Matrix *m_s_multiply(Matrix*m,float s)
{
    int i,j;
    Matrix *temp = m_constructor(local, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            temp->triMatrix[i][j] = m->triMatrix[i][j] * s;
        }
    }    
    return temp;   
};

float m_determinant(Matrix *m)
{
    return m->triMatrix[0][0] * m->triMatrix[1][1] * m->triMatrix[2][2] + m->triMatrix[1][0] * m->triMatrix[2][1] * m->triMatrix[0][2] + m->triMatrix[2][0] * m->triMatrix[0][1] * m->triMatrix[1][2] -
		m->triMatrix[2][0] * m->triMatrix[1][1] * m->triMatrix[0][2] - m->triMatrix[1][0] * m->triMatrix[0][1] * m->triMatrix[2][2] - m->triMatrix[0][0] * m->triMatrix[2][1] * m->triMatrix[1][2];
};

Matrix *m_cofactor(Matrix *m)
{
    Matrix *temp = m_constructor(local, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    temp->triMatrix[0][0] = m->triMatrix[1][1] * m->triMatrix[2][2]- m->triMatrix[1][2] * m->triMatrix[2][1];
	temp->triMatrix[0][1] = -(m->triMatrix[0][1] * m->triMatrix[2][2] - m->triMatrix[2][1] * m->triMatrix[0][2]);
	temp->triMatrix[0][2] = m->triMatrix[0][1] * m->triMatrix[1][2] - m->triMatrix[1][1] * m->triMatrix[0][2];
	temp->triMatrix[1][0] = -(m->triMatrix[1][0] * m->triMatrix[2][2] - m->triMatrix[2][0] * m->triMatrix[1][2]);
	temp->triMatrix[1][1] = m->triMatrix[0][0] * m->triMatrix[2][2] - m->triMatrix[2][0] * m->triMatrix[0][2];
	temp->triMatrix[1][2] = -(m->triMatrix[0][0] * m->triMatrix[1][2] - m->triMatrix[1][0] * m->triMatrix[0][2]);
	temp->triMatrix[2][0] = m->triMatrix[1][0] * m->triMatrix[2][1] - m->triMatrix[2][0] * m->triMatrix[1][1]; 
	temp->triMatrix[2][1] = -(m->triMatrix[0][0] * m->triMatrix[2][1] - m->triMatrix[2][0] * m->triMatrix[0][1]);
	temp->triMatrix[2][2] = m->triMatrix[0][0] * m->triMatrix[1][1] - m->triMatrix[1][0] * m->triMatrix[0][1];

	return temp;
};

Matrix *m_inverse(Matrix *m)
{
    Matrix *temp = m_constructor(local, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    float deter = m_determinant(m);
    /*temp = m_cofactor(m)->m_s_multiply(m_cofactor(m),1/deter);*/
	temp = m_s_multiply(m_cofactor(m), 1 / deter);
    return temp;
}

void m_equal(Matrix*m,Matrix*n)
{
    int i,j;
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            m->triMatrix[i][j] = n->triMatrix[i][j];
        }
    }    
//    return m;  
}