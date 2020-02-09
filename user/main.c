#include "task.h"
#include "usart.h"
#include "iwdg.h"
#include "pwm_out.h"
#include "systick.h"

//定时任务的各相应标志位
#define TASK_1ms		0x01
#define TASK_20ms		0x02
#define TASK_50ms		0x04
#define TASK_100ms		0x08
#define TASK_200ms		0x10
#define TASK_500ms		0x20

u8 Time1ms=0;   //每1ms增1
u8 Time100ms=0; //每100ms增1
u8 TaskFlag=0;  //定时任务的标志位寄存器

int main(void)
{
/*初始化*/
	PWM_OUT_Configuration();  //上电后先输出控制信号
	Para_Init();  //设置油门为最低点、舵机归中，以及其它参数设置
	IWDG_Init();  //若初始化失败可复位重新初始化
	(MPU_Init());  //若MPU6050初始化失败则复位重启
	PWM_IN_Configuration();
	SysTick_Init();
	uart1_init(38400);  //数传的波特率要单独设置，所以尽量不要改波特率
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	DMA_Config();
	LED_GPIO_Config();
	LED_SELECT(AircraftMode);  //初始化完成后，机型对应的指示灯亮
/*死循环*/
 	while(1)
	{
		if(TaskFlag & TASK_1ms)
		{
			IWDG_ReloadCounter();  //喂狗：重新装载计数器
			TaskFlag&=~TASK_1ms;
		}
		if(TaskFlag & TASK_20ms)
		{
			RC_Prepare();
			IMU_Processing();
			Motor_Iner_loop();
			Send_Data_To_DMA_20ms();
			DMA_Enable();
			TaskFlag&=~TASK_20ms;
		}
		if(TaskFlag & TASK_50ms)
		{
			Motor_Outer_loop();
			Send_Data_To_DMA_50ms();
			TaskFlag&=~TASK_50ms;
		}
		if(TaskFlag & TASK_100ms)
		{
			Lock_And_Unlock();
			PID_Set_Parameter();
			TaskFlag&=~TASK_100ms;
		}
		if(TaskFlag & TASK_200ms)
		{
			Send_Data_To_DMA_200ms();
			TaskFlag&=~TASK_200ms;
		}
		if(TaskFlag & TASK_500ms)
		{
			TaskFlag&=~TASK_500ms;
		}
	}
}

/***********************
1ms中断一次
*避免使每个任务的开始时刻为整数倍关系，
这样可以避免同一时刻执行多个任务，保证计时准确
**********************/
void SysTick_Handler(void)
{
	TaskFlag|=TASK_1ms;
	switch(Time1ms)
	{
		case 5:
		case 25:
		case 45:
		case 65:
		case 85:TaskFlag|=TASK_20ms;break;
		case 10:
		case 60:TaskFlag|=TASK_50ms;break;
		case 100:
			TaskFlag|=TASK_100ms;
			Time100ms++;
			Time1ms=0;
			break;
		default:break;
	}
	Time1ms++;
	if(Time1ms>100)
		Time1ms=0;
	if(Time100ms & 0x01)
		TaskFlag|=TASK_200ms;
	if(Time100ms>=5)
	{
		TaskFlag|=TASK_500ms;
		Time100ms=0;
	}
}
