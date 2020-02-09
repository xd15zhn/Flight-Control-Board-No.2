#include "gpio.h"

void LED_GPIO_Config(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(LED_GPIO_RCC,ENABLE);//使能GPIO的外设时钟
	
	GPIO_InitStructure.GPIO_Pin =LED0_GPIO_PIN|LED1_GPIO_PIN|LED2_GPIO_PIN;//选择要用的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //设置引脚模式为推挽输出模式						 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//设置引脚速度为50MHZ         
	GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);//调用库函数，初始化GPIO
}

void LED_SELECT(u8 AircraftMode)
{
	switch(AircraftMode)
	{
		case 1: LED_ONOFF(Bit_RESET,LED0_GPIO_PIN);  //D1 is enabled which means the module is'SIZHOU'
				LED_ONOFF(Bit_SET,LED1_GPIO_PIN);
				LED_ONOFF(Bit_SET,LED2_GPIO_PIN);
		break;

		case 2: LED_ONOFF(Bit_SET,LED0_GPIO_PIN);    //D3 is enabled which means the module is'YUYING'
				LED_ONOFF(Bit_SET,LED1_GPIO_PIN);
				LED_ONOFF(Bit_RESET,LED2_GPIO_PIN);
		break;
		
		default : LED_ONOFF(Bit_SET,LED0_GPIO_PIN);       //No one is enabled which means the module is unkown
				LED_ONOFF(Bit_SET,LED1_GPIO_PIN);
				LED_ONOFF(Bit_SET,LED2_GPIO_PIN);
		break;					
	}
}
