#include "adrc.h"
/**************文件说明**********************
前半部分为自抗扰控制器相关函数,后半部分为控制飞行器的4个函数
与task.c共享头文件task.h
--------------飞行器安装与控制---------------
F450四轴，X型，红前白后，从右前方电机编号为1开始逆时针编号
********************************************/

/**********************
离散系统最速控制综合函数
**********************/
float ADRC_fhan(float x1,float x2)
{
	float d=0.0004;
	float a0=0.02*x2;
	float y=x1+a0;
	float a1=Msqrt(d*(d+8*ABS(y)));
	float a2=a0+SIGN(y)*(a1-d)/2;
	float sy=(SIGN(y+d)-SIGN(y-d))/2;
	float a=(a0+y-a2)*sy+a2;
	float sa=(SIGN(a+d)-SIGN(a-d))/2;
	y=-5*(a/d-SIGN(a))*sa-5*SIGN(a);
	return y;
}

/**********************
跟踪微分器
**********************/
void ADRC_TD(float x,float *track,float *derivative)
{
	float u=ADRC_fhan(*track-x,*derivative);
	*derivative+=u;
	*track+=*derivative;
}

/**********************
非线性增益
**********************/
float ADRC_fal(float x)
{
	float y;
	if(ABS(x)<=0.02)
		y=x*0.1414213562;
	else
		y=Msqrt(ABS(x))*SIGN(x);
	return y;
}

/**********************
扩张状态观测器
**********************/
float ADRC_ESO(float u,float y,float b)
{
	static float z1=0,z2=0;
	float e=z1-y;
	z1+=b*u+z2-50*e;
	z2-=220*ADRC_fal(e);
	float w=z2/b;
	return w;
}

/**********************
求出姿态四元数pos到期望四元数exp之间的误差四元数
**********************/
Quaternion Quaternion_Error(Quaternion E,Quaternion P)
{
	Quaternion ans;
	ans.q0=E.q0*P.q0+E.q1*P.q1+E.q2*P.q2+E.q3*P.q3;
	ans.q1=P.q0*E.q1-E.q0*P.q1+E.q3*P.q2-E.q2*P.q3;
	ans.q2=P.q0*E.q2-E.q0*P.q2+E.q1*P.q3-E.q3*P.q1;
	ans.q3=P.q0*E.q3-E.q0*P.q3+E.q2*P.q1-E.q1*P.q2;
	return ans;
}
