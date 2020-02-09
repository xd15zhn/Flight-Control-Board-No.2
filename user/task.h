#ifndef __TASK_H
#define __TASK_H

#include "niming.h"
#include "imu.h"
#include "mpu6050.h"
#include "adrc.h"
#include "pwm_in.h"
#include "gpio.h"

#define OUTPUT1  (TIM3->CCR1) 
#define OUTPUT2  (TIM3->CCR2)
#define OUTPUT3  (TIM3->CCR3)
#define OUTPUT4  (TIM3->CCR4)
#define OUTPUT5  (TIM4->CCR1)
#define OUTPUT6  (TIM4->CCR2)
#define OUTPUT7  (TIM4->CCR3)
#define OUTPUT8  (TIM4->CCR4)

#define LOWSPEED          1100	 //怠速
#define LOW_THRESHOLD     1250  //5和7通道的1,2档阈值
#define HIGH_THRESHOLD    1750  //5和7通道的2,3档阈值
#define LOCK_HIGH         1800  //锁定和解锁摇杆上阈值
#define LOCK_LOW          1200  //锁定和解锁摇杆下阈值
#define LOW_SERVO         500  //舵机行程下极限
#define HIGH_SERVO        2500  //舵机行程上极限
//LockMode
#define LOCKED    0	//锁定状态且无操作
#define TOUNLOCK  1	//锁定状态且尝试解锁
#define UNLOCKED  2	//解锁状态
#define TIME      20	//解锁时间,2秒
//WholeCommand
#define NORMAL_WORK     0x01  //高电平表示自检完毕，进入正常工作状态
#define PREPARE         0x02  //高电平脉冲触发校准
#define CALIBRATING     0x04  //高电平表示正在校准
#define CALIBRATED      0x08  //高电平表示校准完毕
//AircraftMode
#define AircraftMode  SIZHOU
#define SIZHOU        1  //四旋翼
#define YUYING        2  //鱼鹰

typedef struct
{
	float KpOut;  //外环比例控制
	float KpIn;  //内环比例控制
	float b;  //扰动补偿增益
}ADRC_Param;

extern u8 Armed;
extern short PwmInTemp[7];
extern AxisInt acc;
extern AxisInt gyro;
extern ADRC_Param RollParam,PitchParam;
extern Quaternion Qpos,Qexp;

//在task.c中
void Para_Init(void);
void Lock_And_Unlock(void);
void IMU_Processing(void);
void RC_Prepare(void);
void Send_Data_To_DMA_20ms(void);
void Send_Data_To_DMA_50ms(void);
void Send_Data_To_DMA_200ms(void);
void PID_Set_Parameter(void);
void Self_Test(void);
//在飞行器控制程序中
void Para_Init(void);
void Motor_Iner_loop(void);
void Motor_Outer_loop(void);

#endif
