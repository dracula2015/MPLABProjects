/*
 * File:   vector.c
 * Author: dracula
 *
 * Created on December 28, 2016, 10:26 PM
 */

#include "user.h"

int memFreeCount = 0;
int countMatrix = 0;
int countVector = 0;
int countMatrixGlobal = 0;
int countVectorGlobal = 0;

Matrix* pointerMatrix[100];
Matrix* pointerMatrixGlobal[100];
Vector3f* pointerVector[100];
Vector3f* pointerVectorGlobal[100];

void v_destructor(Vector3f*v, bool dynamic)
{
    if(dynamic)
    {
		if (v->globalVector == false)
		{
			free(v);
		}
    }
}

Vector3f *v_plus(Vector3f*v,Vector3f*w)
{
    return v_constructor(local, NULL, v->x + w->x, v->y + w->y, v->z + w->z);
}

Vector3f *v_minus(Vector3f*v,Vector3f*w)
{
    return v_constructor(local, NULL, v->x - w->x, v->y - w->y, v->z - w->z);
}

float v_v_multiply(Vector3f*v,Vector3f*w)
{
    return v->x * w->x + v->y * w->y + v->z * w->z;
}

Vector3f *v_s_multiply(Vector3f*v,float s)
{
    return v_constructor(local, NULL, v->x * s, v->y * s, v->z * s);
}

void v_equal(Vector3f*v,Vector3f*w)
{
    v->x = w->x;
    v->y = w->y;
    v->z = w->z;
//    return v;
}

float v_length(Vector3f*v)
{
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

void v_normalize(Vector3f*v)
{
    float len = v_length(v);
    v->x /= len;
    v->y /= len;
    v->z /= len;
}

Vector3f *v_constructor(bool globalVector, Vector3f*v, float x, float y, float z)
{
    if(v == NULL)
    {
		if(!globalVector)
		{
			pointerVector[countVector] = (Vector3f*)malloc(sizeof(Vector3f));
			v = pointerVector[countVector];
			v->thisVectorAddress = countVector;
			countVector++;
		}
		else 
		{
			pointerVectorGlobal[countVectorGlobal] = (Vector3f*)malloc(sizeof(Vector3f));
			v = pointerVectorGlobal[countVectorGlobal];
			v->thisVectorAddress = countVectorGlobal;
			countVectorGlobal++;
		}
    }

	v->globalVector = globalVector;
    v->x = x;
    v->y = y;
    v->z = z;
    //v->v_plus = v_plus;
    //v->v_minus = v_minus;
    //v->v_v_multiply = v_v_multiply;
    //v->v_s_multiply = v_s_multiply;
    //v->v_equal = v_equal;
    //v->v_length = v_length;
    //v->v_normalize = v_normalize;
    return v;
}

void freeLocalMem(void)
{
    for (memFreeCount = 0; memFreeCount < countMatrix; memFreeCount++)
    {
        m_destructor(pointerMatrix[memFreeCount], 1);
    };
    for (memFreeCount = 0; memFreeCount < countVector; memFreeCount++)
    {
        v_destructor(pointerVector[memFreeCount], 1);
    }
    countMatrix = 0;
    countVector = 0;
}

void freeGlobalMem(void)
{
    for (memFreeCount = 0; memFreeCount < countMatrixGlobal; memFreeCount++)
	{
		m_destructor(pointerMatrixGlobal[memFreeCount], 1);
	};
	for (memFreeCount = 0; memFreeCount < countVectorGlobal; memFreeCount++)
	{
		v_destructor(pointerVectorGlobal[memFreeCount], 1);
	}
	countMatrixGlobal = 0;
	countVectorGlobal = 0;
}