#include "iwdg.h"
void IWDG_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对寄存器IWDG_PR、IWDG_RLR的写操作
	IWDG_SetPrescaler(IWDG_Prescaler_32);  //设置IWDG预分频值：256分频最大
	IWDG_SetReload(40000/128);  //设置IWDG的重装载值	:范围0~0x0FFF
	IWDG_ReloadCounter();  //喂狗：重新装载计数器
	IWDG_Enable();  //使能IWDG定时器
}//250ms
