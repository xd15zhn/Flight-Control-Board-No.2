#include "pwm_in.h"

short PwmIn[8];
u8 TIM2Capture_STA=0;		//低四位标志捕获状态
u8 TIM8Capture_STA=0;		//低四位标志捕获状态
u16 Rise[8]={0};
u16 Drop[8]={0};
u16 t2sr;		//定时器2状态寄存器
u16 t8sr;		//定时器4状态寄存器
void TIM2_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}
void TIM2_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period =0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_Cmd(TIM2, ENABLE);
}
void TIM2_Input_config(void)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
}
void TIM8_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8 | GPIO_Pin_9;     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}
void TIM8_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period =0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  TIM_Cmd(TIM8, ENABLE);
}
void TIM8_Input_config(void)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);
}
void NVIC_Capture_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void PWM_IN_Configuration(void)
{
  TIM2_GPIO_Config();
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
  TIM2_Mode_Config();
  TIM2_Input_config();

  TIM8_GPIO_Config();
  TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_CC3, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_CC4, ENABLE);
  TIM8_Mode_Config();
  TIM8_Input_config();
  NVIC_Capture_Config();
	PwmIn[0]=1000;
	PwmIn[1]=1000;
	PwmIn[2]=1000;
	PwmIn[3]=1000;
	PwmIn[4]=1000;
	PwmIn[5]=1000;
	PwmIn[6]=1000;
}
void TIM2_IRQHandler(void) 
{
	t2sr=TIM2->SR;
	//CH1中断
	if(t2sr&0x02)
	{
		if(TIM2Capture_STA&0x01)		//捕获到一个下降沿
		{
			Drop[0]=TIM2->CCR1;		//获取当前捕获值
			if(Rise[0]<=Drop[0])
				PwmIn[0]=Drop[0]-Rise[0];
			else
				PwmIn[0]=0xFFFF-Rise[0]+Drop[0];
			
			Rise[0]=0;
			Drop[0]=0;
			TIM2Capture_STA&=~0x01;		//清零
			TIM2->CCER&=~(1<<1);		//设置为上升沿捕获
		}
		else		//第一次捕获上升沿
		{
			TIM2Capture_STA|=0x01;		//标记捕获到一个上升沿
			Rise[0]=TIM2->CCR1;		//获取当前捕获值
			TIM2->CCER|=(1<<1);		//设置为下降沿捕获
		}
	}
	//CH2中断
	if(t2sr&0x04)
	{
		if(TIM2Capture_STA&0x02)		//捕获到一个下降沿
		{
			Drop[1]=TIM2->CCR2;		//获取当前捕获值
			if(Rise[1]<=Drop[1])
				PwmIn[1]=Drop[1]-Rise[1];
			else
				PwmIn[1]=0xFFFF-Rise[1]+Drop[1];
			
			Rise[1]=0;
			Drop[1]=0;
			TIM2Capture_STA&=~0x02;		//清零
			TIM2->CCER&=~(1<<5);		//设置为上升沿捕获
		}
		else		//第一次捕获上升沿
		{
			TIM2Capture_STA|=0x02;		//标记捕获到一个上升沿
			Rise[1]=TIM2->CCR2;		//获取当前捕获值
			TIM2->CCER|=(1<<5);		//设置为下降沿捕获
		}
	}
	//CH3中断
	if(t2sr&0x08)
	{
		if(TIM2Capture_STA&0x04)		//捕获到一个下降沿
		{
			Drop[2]=TIM2->CCR3;		//获取当前捕获值
			if(Rise[2]<=Drop[2])
				PwmIn[2]=Drop[2]-Rise[2];
			else
				PwmIn[2]=0xFFFF-Rise[2]+Drop[2];
			
			Rise[2]=0;
			Drop[2]=0;
			TIM2Capture_STA&=~0x04;		//清零
			TIM2->CCER&=~(1<<9);		//设置为上升沿捕获
		}
		else		//第一次捕获上升沿
		{
			TIM2Capture_STA|=0x04;		//标记捕获到一个上升沿
			Rise[2]=TIM2->CCR3;		//获取当前捕获值
			TIM2->CCER|=(1<<9);		//设置为下降沿捕获
		}
	}
	//CH4中断
	if(t2sr&0x10)
	{
		if(TIM2Capture_STA&0x08)		//捕获到一个下降沿
		{
			Drop[3]=TIM2->CCR4;		//获取当前捕获值
			if(Rise[3]<=Drop[3])
				PwmIn[3]=Drop[3]-Rise[3];
			else
				PwmIn[3]=0xFFFF-Rise[3]+Drop[3];
			
			Rise[3]=0;
			Drop[3]=0;
			TIM2Capture_STA&=~0x08;		//清零
			TIM2->CCER&=~(1<<13);		//设置为上升沿捕获
		}
		else		//第一次捕获上升沿
		{
			TIM2Capture_STA|=0x08;		//标记捕获到一个上升沿
			Rise[3]=TIM2->CCR4;		//获取当前捕获值
			TIM2->CCER|=(1<<13);		//设置为下降沿捕获
		}
	}
	TIM2->SR=0;
}

void TIM8_CC_IRQHandler(void) 
{
	t8sr=TIM8->SR;
	//CH1中断
	if(t8sr&0x02)
	{
		if(TIM8Capture_STA&0x01)		//捕获到一个下降沿
		{
			Drop[4]=TIM8->CCR1;		//获取当前捕获值
			if(Rise[4]<=Drop[4])
				PwmIn[4]=Drop[4]-Rise[4];
			else
				PwmIn[4]=0xFFFF-Rise[4]+Drop[4];
			
			Rise[4]=0;
			Drop[4]=0;
			TIM8Capture_STA&=~0x01;		//清零
			TIM8->CCER&=~(1<<1);		//设置为上升沿捕获
		}
		else		//第一次捕获上升沿
		{
			TIM8Capture_STA|=0x01;		//标记捕获到一个上升沿
			Rise[4]=TIM8->CCR1;		//获取当前捕获值
			TIM8->CCER|=(1<<1);		//设置为下降沿捕获
		}
	}
	//CH2中断
	if(t8sr&0x04)
	{
		if(TIM8Capture_STA&0x02)		//捕获到一个下降沿
		{
			Drop[5]=TIM8->CCR2;		//获取当前捕获值
			if(Rise[5]<=Drop[5])
				PwmIn[5]=Drop[5]-Rise[5];
			else
				PwmIn[5]=0xFFFF-Rise[5]+Drop[5];
			
			Rise[5]=0;
			Drop[5]=0;
			TIM8Capture_STA&=~0x02;		//清零
			TIM8->CCER&=~(1<<5);		//设置为上升沿捕获
		}
		else		//第一次捕获上升沿
		{
			TIM8Capture_STA|=0x02;		//标记捕获到一个上升沿
			Rise[5]=TIM8->CCR2;		//获取当前捕获值
			TIM8->CCER|=(1<<5);		//设置为下降沿捕获
		}
	}
	//CH3中断
	if(t8sr&0x08)
	{
		if(TIM8Capture_STA&0x04)		//捕获到一个下降沿
		{
			Drop[6]=TIM8->CCR3;		//获取当前捕获值
			if(Rise[6]<=Drop[6])
				PwmIn[6]=Drop[6]-Rise[6];
			else
				PwmIn[6]=0xFFFF-Rise[6]+Drop[6];
			
			Rise[6]=0;
			Drop[6]=0;
			TIM8Capture_STA&=~0x04;		//清零
			TIM8->CCER&=~(1<<9);		//设置为上升沿捕获
		}
		else		//第一次捕获上升沿
		{
			TIM8Capture_STA|=0x04;		//标记捕获到一个上升沿
			Rise[6]=TIM8->CCR3;		//获取当前捕获值
			TIM8->CCER|=(1<<9);		//设置为下降沿捕获
		}
	}
	//CH4中断
	if(t8sr&0x10)
	{
		if(TIM8Capture_STA&0x08)		//捕获到一个下降沿
		{
			Drop[7]=TIM8->CCR4;		//获取当前捕获值
			if(Rise[7]<=Drop[7])
				PwmIn[7]=Drop[7]-Rise[7];
			else
				PwmIn[7]=0xFFFF-Rise[7]+Drop[7];
			
			Rise[7]=0;
			Drop[7]=0;
			TIM8Capture_STA&=~0x08;		//清零
			TIM8->CCER&=~(1<<13);		//设置为上升沿捕获
		}
		else		//第一次捕获上升沿
		{
			TIM8Capture_STA|=0x08;		//标记捕获到一个上升沿
			Rise[7]=TIM8->CCR4;		//获取当前捕获值
			TIM8->CCER|=(1<<13);		//设置为下降沿捕获
		}
	}
	TIM8->SR=0;
}
