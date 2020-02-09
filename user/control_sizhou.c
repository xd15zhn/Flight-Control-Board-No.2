#include "task.h"
/**************文件说明**********************
Para_Init();
Motor_Iner_loop();
Motor_Outer_loop();
与task.c共享头文件task.h
--------------飞行器安装与控制---------------
F450四轴，X型，红前白后，从右前方电机编号为1开始逆时针编号
********************************************/
short PwmOut[8]={1000,1000,1000,1000,1000,1000,1000,1000};  //把值赋给定时器，输出PWM
float RollExp=0,PitchExp=0;
float RollOut=0,PitchOut=0;

/**********************
控制参数初始化
必须在上电后至少一个PWM周期后进行
**********************/
void Para_Init(void)
{
	LED_SELECT(AircraftMode);
	OUTPUT1=PwmOut[0];OUTPUT2=PwmOut[1];OUTPUT3=PwmOut[2];OUTPUT4=PwmOut[3];
	OUTPUT5=PwmOut[4];OUTPUT6=PwmOut[5];OUTPUT7=PwmOut[6];OUTPUT8=PwmOut[7];
	RollParam.KpOut=1.5;RollParam.KpIn=0.5;RollParam.b=100;
	PitchParam.KpOut=1.5;PitchParam.KpIn=0.5;PitchParam.b=100;
}

/**********************
电机内环自抗扰控制
**********************/
void Motor_Iner_loop(void)
{
	if(!Armed)
	{
		OUTPUT1=1000;
		OUTPUT2=1000;
		OUTPUT3=1000;
		OUTPUT4=1000;
		return;
	}
	if(PwmInTemp[2]<LOWSPEED)
	{
		OUTPUT1=LOWSPEED;
		OUTPUT2=LOWSPEED;
		OUTPUT3=LOWSPEED;
		OUTPUT4=LOWSPEED;
		return;
	}
	float gx=GyroToDeg(gyro.x);
	float gy=GyroToDeg(gyro.y);
	float RollErr=RollExp-gx;
	float PitchErr=PitchExp-gy;
	RollOut=ADRC_fal(RollParam.KpIn*RollErr)-ADRC_ESO(RollOut,gx,RollParam.b);
	PitchOut=ADRC_fal(PitchParam.KpIn*PitchErr)-ADRC_ESO(PitchOut,gy,PitchParam.b);
	PwmOut[0]=DegToPwmAdd(+RollOut+PitchOut);
	PwmOut[1]=DegToPwmAdd(-RollOut+PitchOut);
	PwmOut[2]=DegToPwmAdd(-RollOut-PitchOut);
	PwmOut[3]=DegToPwmAdd(+RollOut-PitchOut);
	OUTPUT1=VAL_LIMIT(PwmOut[0],LOWSPEED,2000);
	OUTPUT2=VAL_LIMIT(PwmOut[1],LOWSPEED,2000);
	OUTPUT3=VAL_LIMIT(PwmOut[2],LOWSPEED,2000);
	OUTPUT4=VAL_LIMIT(PwmOut[3],LOWSPEED,2000);
}

/**********************
电机外环非线性比例控制
**********************/
void Motor_Outer_loop(void)
{
	Quaternion Qerr=Quaternion_Error(Qexp,Qpos);
	RollExp=RadToDeg(Masin(Qerr.q1));
	PitchExp=RadToDeg(Masin(Qerr.q2));
	RollExp=ADRC_fal(RollParam.KpOut*RollExp);
	PitchExp=ADRC_fal(PitchParam.KpOut*PitchExp);
}
