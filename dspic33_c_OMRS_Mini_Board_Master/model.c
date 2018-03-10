/*
* File:   model.c
* Author: dracula
*
* Created on December 18, 2017, 1:29 AM
*/

#include "user.h"
extern Parameter P;
Vector3f *OMRS_model(Vector3f *u, Vector3f *q, Vector3f *dq)
{
#ifdef printTimeConsumption
	int modelInitTime = clock();
#endif // printTimeConsumption

	Vector3f *ddq;
	//Matrix *Ravc = m_constructor(local, NULL, NULL, cos(q->z), -sin(q->z), 0, sin(q->z), cos(q->z), 0, 0, 0, 1);
	Matrix *DRavc = m_constructor(local, NULL, NULL, -sin(q->z)*dq->z, -cos(q->z)*dq->z, 0, cos(q->z)*dq->z, -sin(q->z)*dq->z, 0, 0, 0, 0);
	Matrix *M2avc = m_constructor(local, NULL, NULL, 1.5*P.beta0 + P.m, 0, 0, 0, 1.5*P.beta0 + P.m, 0, 0, 0, 3 * P.beta0*pow(P.La, 2) + P.Iv);
	Matrix *C2avc = m_constructor(local, NULL, NULL, 1.5*P.beta1, -P.m*dq->z, 0, P.m*dq->z, 1.5*P.beta1, 0, 0, 0, 3 * P.beta1*pow(P.La, 2));
	Matrix *RavcRev = m_constructor(local, NULL, NULL, cos(q->z), sin(q->z), 0, -sin(q->z), cos(q->z), 0, 0, 0, 1);
	Matrix *Mavc = m_m_multiply(M2avc, RavcRev);
	Matrix *Cavc = m_minus(m_m_multiply(C2avc, RavcRev), m_m_multiply(M2avc, m_m_multiply(RavcRev, m_m_multiply(DRavc, RavcRev))));
	Matrix *BavcOri = m_constructor(local, NULL, NULL, -0.5, -0.5, 1, 0.866, -0.866, 0, P.La, P.La, P.La);
	Matrix *Bavc = m_s_multiply(BavcOri, P.beta2);
	ddq = m_v_multiply(m_inverse(Mavc), v_minus(m_v_multiply(Bavc, u), m_v_multiply(Cavc, dq)));

#ifdef printDetail
    printf("ddq: %f,%f,%f\n", ddq->x, ddq->y, ddq->z);
	printf("Ravc: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		Ravc->triMatrix[0][0], Ravc->triMatrix[0][1], Ravc->triMatrix[0][2],
		Ravc->triMatrix[1][0], Ravc->triMatrix[1][1], Ravc->triMatrix[1][2],
		Ravc->triMatrix[2][0], Ravc->triMatrix[2][1], Ravc->triMatrix[2][2]);
	printf("DRavc: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		DRavc->triMatrix[0][0], DRavc->triMatrix[0][1], DRavc->triMatrix[0][2],
		DRavc->triMatrix[1][0], DRavc->triMatrix[1][1], DRavc->triMatrix[1][2],
		DRavc->triMatrix[2][0], DRavc->triMatrix[2][1], DRavc->triMatrix[2][2]);
	printf("M2avc: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		M2avc->triMatrix[0][0], M2avc->triMatrix[0][1], M2avc->triMatrix[0][2],
		M2avc->triMatrix[1][0], M2avc->triMatrix[1][1], M2avc->triMatrix[1][2],
		M2avc->triMatrix[2][0], M2avc->triMatrix[2][1], M2avc->triMatrix[2][2]);
	printf("C2avc: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		C2avc->triMatrix[0][0], C2avc->triMatrix[0][1], C2avc->triMatrix[0][2],
		C2avc->triMatrix[1][0], C2avc->triMatrix[1][1], C2avc->triMatrix[1][2],
		C2avc->triMatrix[2][0], C2avc->triMatrix[2][1], C2avc->triMatrix[2][2]);
	printf("RavcRev: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		RavcRev->triMatrix[0][0], RavcRev->triMatrix[0][1], RavcRev->triMatrix[0][2],
		RavcRev->triMatrix[1][0], RavcRev->triMatrix[1][1], RavcRev->triMatrix[1][2],
		RavcRev->triMatrix[2][0], RavcRev->triMatrix[2][1], RavcRev->triMatrix[2][2]);
	printf("Mavc: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		Mavc->triMatrix[0][0], Mavc->triMatrix[0][1], Mavc->triMatrix[0][2],
		Mavc->triMatrix[1][0], Mavc->triMatrix[1][1], Mavc->triMatrix[1][2],
		Mavc->triMatrix[2][0], Mavc->triMatrix[2][1], Mavc->triMatrix[2][2]);
	printf("Cavc: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		Cavc->triMatrix[0][0], Cavc->triMatrix[0][1], Cavc->triMatrix[0][2],
		Cavc->triMatrix[1][0], Cavc->triMatrix[1][1], Cavc->triMatrix[1][2],
		Cavc->triMatrix[2][0], Cavc->triMatrix[2][1], Cavc->triMatrix[2][2]);
	printf("BavcOri: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		BavcOri->triMatrix[0][0], BavcOri->triMatrix[0][1], BavcOri->triMatrix[0][2],
		BavcOri->triMatrix[1][0], BavcOri->triMatrix[1][1], BavcOri->triMatrix[1][2],
		BavcOri->triMatrix[2][0], BavcOri->triMatrix[2][1], BavcOri->triMatrix[2][2]);
	printf("Bavc: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		Bavc->triMatrix[0][0], Bavc->triMatrix[0][1], Bavc->triMatrix[0][2],
		Bavc->triMatrix[1][0], Bavc->triMatrix[1][1], Bavc->triMatrix[1][2],
		Bavc->triMatrix[2][0], Bavc->triMatrix[2][1], Bavc->triMatrix[2][2]);

	Matrix *tm1 = m_m_multiply(C2avc, RavcRev);
	Matrix *tm2 = m_m_multiply(M2avc, RavcRev);
	Matrix *tm3 = m_m_multiply(M2avc, m_m_multiply(RavcRev, m_m_multiply(DRavc, RavcRev)));
	printf("C2avc*RavcRev\': %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		tm1->triMatrix[0][0], tm1->triMatrix[0][1], tm1->triMatrix[0][2],
		tm1->triMatrix[1][0], tm1->triMatrix[1][1], tm1->triMatrix[1][2],
		tm1->triMatrix[2][0], tm1->triMatrix[2][1], tm1->triMatrix[2][2]);
	printf("M2avc*Ravc\'*DRavc*Ravc\': %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		tm2->triMatrix[0][0], tm2->triMatrix[0][1], tm2->triMatrix[0][2],
		tm2->triMatrix[1][0], tm2->triMatrix[1][1], tm2->triMatrix[1][2],
		tm2->triMatrix[2][0], tm2->triMatrix[2][1], tm2->triMatrix[2][2]);
	printf("M2avc*Ravc\'*DRavc*Ravc\': %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
		tm3->triMatrix[0][0], tm3->triMatrix[0][1], tm3->triMatrix[0][2],
		tm3->triMatrix[1][0], tm3->triMatrix[1][1], tm3->triMatrix[1][2],
		tm3->triMatrix[2][0], tm3->triMatrix[2][1], tm3->triMatrix[2][2]);
#endif // printDetail

#ifdef printTimeConsumption
	printf("model time: %d\n", (clock() - modelInitTime));
#endif // printTimeConsumption

	return ddq;
}