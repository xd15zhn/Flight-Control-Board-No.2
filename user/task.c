#include "task.h"
/**************文件说明**********************
除了飞行器控制函数之外的定时函数，分别为：
Lock_And_Unlock();             锁定，解锁
RC_Prepare();                  对接收机的信号进行预处理
IMU_Processing();              姿态解算更新，MPU6050数据校准
Send_Data_To_DMA_20ms();       往DMA缓存中填入数据等待发给地面站
Send_Data_To_DMA_50ms();       往DMA缓存中填入数据等待发给地面站
Send_Data_To_DMA_200ms();      往DMA缓存中填入数据等待发给地面站
PID_Set_Parameter();           PID等参数设置
********************************************/

Quaternion Qpos={1,0,0,0},Qexp;  //姿态四元数和期望四元数
AxisInt acc;  //三轴加速度
AxisInt gyro;  //三轴角速度
AxisInt oacc;  //三轴加速度计原始数据
AxisInt ogyro;  //三轴陀螺仪原始数据
AxisInt Bias;  //手动模式初始误差补偿,用于固定翼的手动模式和所有旋翼,可代替遥控器的微调
//AxisInt AutoBias;  //自动模式初始误差补偿，用于固定翼的半自动和全自动模式
u8 Armed=0;  //默认锁定
short PwmInTemp[7]={1000,1000,1000,1000,1000,1000,1000};  //防止在运算过程中被中断所更改(临界代码保护)
ADRC_Param RollParam,PitchParam;

/***********************
*@function:锁定，解锁
*@period:100ms
*@note:左操纵杆推到右下方解锁;左操纵杆推到左下方锁定
**********************/
void Lock_And_Unlock(void)
{
	static u8 LockMode=LOCKED;//锁定/解锁过程状态机
	static u16 t=0;//解锁过程需要的时间
	if((acc.z<15000)&&(LockMode==LOCKED)){Armed=0;return;}//传感器异常
	if(AircraftMode&0x80){LockMode=UNLOCKED;Armed=1;return;}//固定翼的免死令牌
	switch(LockMode)
	{
		case LOCKED:
			if((PwmInTemp[0]>LOCK_HIGH)&&(PwmInTemp[1]<LOCK_LOW)&&(PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]<LOCK_LOW))//自检完成并且有解锁操作
			{LockMode=TOUNLOCK;t++;}
			break;
		case TOUNLOCK:
			if(t>TIME)//达到解锁时间后解锁
			{LockMode=UNLOCKED;t=0;Armed=1;}
			else if((PwmInTemp[0]>LOCK_HIGH)&&(PwmInTemp[1]<LOCK_LOW)&&(PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]<LOCK_LOW))//依然在尝试解锁
				t++;
			else//放弃解锁
			{LockMode=LOCKED;t=0;}
			break;
		case UNLOCKED:
			if((PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]>LOCK_HIGH))//有锁定操作
			{LockMode=LOCKED;Armed=0;}//与解锁操作不同，有锁定操作立即锁定
			break;
		default:break;
	}
}

/***********************
*@function:姿态解算更新,MPU6050数据校准
*@period:20ms
**********************/
void IMU_Processing(void)
{
	static float IIRgyrox[3]={0,0,0};
	static float IIRgyroy[3]={0,0,0};
	static float IIRgyroz[3]={0,0,0};
	MPU_Get_Accelerometer(&acc.x,&acc.y,&acc.z);
	MPU_Get_Gyroscope(&gyro.x,&gyro.y,&gyro.z);
	oacc=acc;ogyro=gyro;
	Acc_Calibrate(&acc);
	IMUupdate(acc,&gyro,&Qpos);
	gyro.x=IIR_LowPassFilter(gyro.x,IIRgyrox);
	gyro.y=IIR_LowPassFilter(gyro.y,IIRgyroy);
	gyro.z=IIR_LowPassFilter(gyro.z,IIRgyroz);
}

/***********************
*@function:对接收机的信号进行预处理并转换为期望四元数
*@period:20ms
**********************/
void RC_Prepare(void)
{
	PwmInTemp[0]=PwmIn[0]-20+Bias.x;
	PwmInTemp[1]=PwmIn[1]-20+Bias.y;
	PwmInTemp[2]=PwmIn[2]-20;
	PwmInTemp[3]=PwmIn[3]-20+Bias.z;
	PwmInTemp[4]=PwmIn[4]-20;
	PwmInTemp[5]=PwmIn[5]-20;
	PwmInTemp[6]=PwmIn[6]-20;
	float Hroll=PwmToRadAdd(PwmInTemp[0])/2;
	float Hpitch=PwmToRadAdd(PwmInTemp[1])/2;
	float Hyaw=PwmToRadAdd(PwmInTemp[3])/2;
	Qexp.q0=Mcos(Hroll)*Mcos(Hpitch)*Mcos(Hyaw)+Msin(Hroll)*Msin(Hpitch)*Msin(Hyaw);
	Qexp.q1=Msin(Hroll)*Mcos(Hpitch)*Mcos(Hyaw)-Mcos(Hroll)*Msin(Hpitch)*Msin(Hyaw);
	Qexp.q2=Mcos(Hroll)*Msin(Hpitch)*Mcos(Hyaw)+Msin(Hroll)*Mcos(Hpitch)*Msin(Hyaw);
	Qexp.q3=Mcos(Hroll)*Mcos(Hpitch)*Msin(Hyaw)-Msin(Hroll)*Msin(Hpitch)*Mcos(Hyaw);
}

/***********************
*@function:往DMA缓存中填入数据等待发给地面站
*@period:20ms
**********************/
void Send_Data_To_DMA_20ms(void)
{
	static u8 count=0;
	s16 mydata1[7];
	mydata1[0]=OUTPUT1;
	mydata1[1]=OUTPUT2;
	mydata1[2]=OUTPUT3;
	mydata1[3]=OUTPUT4;
	mydata1[4]=count;
	if(PwmInTemp[4]>=HIGH_THRESHOLD)
	{
		ANO_Send_User_Data(mydata1,5,0xF1);
		ANO_DT_Send_Senser(acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,0,0,0);
	}
	else if((PwmInTemp[4]<HIGH_THRESHOLD)&&(PwmInTemp[4]>LOW_THRESHOLD))
		ANO_DT_Send_Senser(oacc.x,oacc.y,oacc.z,ogyro.x,ogyro.y,ogyro.z,0,0,0);
	count++;
}

/***********************
*@function:往DMA缓存中填入数据等待发给地面站
*@period:50ms
**********************/
void Send_Data_To_DMA_50ms(void)
{
	float roll=Matan2(2*(Qpos.q0*Qpos.q1+Qpos.q2*Qpos.q3),1-2*(Qpos.q1*Qpos.q1+Qpos.q2*Qpos.q2))*57.3f;
	float pitch=Masin(2*(Qpos.q0*Qpos.q2-Qpos.q1*Qpos.q3))*57.3f;
	float yaw=Matan2(2*(Qpos.q1*Qpos.q2+Qpos.q0*Qpos.q3),1-2*(Qpos.q2*Qpos.q2+Qpos.q3*Qpos.q3))*57.3f;
	if(PwmInTemp[4]>=HIGH_THRESHOLD)
		ANO_DT_Send_Status(roll,pitch,yaw,0,0,Armed);
}

/***********************
*@function:往DMA缓存中填入数据等待发给地面站
*@period:200ms
**********************/
void Send_Data_To_DMA_200ms(void)
{
//	ANO_DT_Send_RCData(PwmInTemp[2],PwmInTemp[3],PwmInTemp[0],PwmInTemp[1],PwmIn[4],PwmIn[5],PwmIn[6],1000,1000,1000);
}

/***********************
*@function:参数设置
*@period:100ms
**********************/
void PID_Set_Parameter(void)
{
	if(ANO_CMD&PID_REQUIRE)//地面站要求读取PID
	{
		ANO_DT_Send_PID(1,RollParam.KpOut,RollParam.KpIn,RollParam.b,PitchParam.KpOut,PitchParam.KpIn,PitchParam.b/100,0,0,0);
		ANO_CMD&=~PID_REQUIRE;
	}
	if(ANO_CMD&PID_SENDBACK)//地面站要求写入PID
	{
		RollParam.KpOut=(PIDReceiveTemp[0][0]*256.0+PIDReceiveTemp[0][1])/1000.0;
		RollParam.KpIn=(PIDReceiveTemp[0][2]*256.0+PIDReceiveTemp[0][3])/1000.0;
		RollParam.b=(PIDReceiveTemp[0][4]*256.0+PIDReceiveTemp[0][5])/10.0;
		PitchParam.KpOut=(PIDReceiveTemp[0][6]*256.0+PIDReceiveTemp[0][7])/1000.0;
		PitchParam.KpIn=(PIDReceiveTemp[0][8]*256.0+PIDReceiveTemp[0][9])/1000.0;
		PitchParam.b=(PIDReceiveTemp[0][10]*256.0+PIDReceiveTemp[0][11])/10.0;

		ANO_CMD&=~PID_SENDBACK;
	}
}
