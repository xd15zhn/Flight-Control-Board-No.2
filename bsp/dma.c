#include "dma.h"

u8 SendBuff[SENDBUF_SIZE];
u16 TotalLen=0;
void NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
//DMA1的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_CHx:DMA通道CHx
//cpar:外设地址
//cmar:存储器地址
//cndtr:数据传输量 
void DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
  DMA_DeInit(DMA1_Channel4);   //将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendBuff;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	NVIC_Config();
}
//开启一次DMA传输
void DMA_Enable(void)
{
	DMA_Cmd(DMA1_Channel4, DISABLE );  //关闭USART1 TX DMA1 所指示的通道      
	DMA1_Channel4->CNDTR=TotalLen;  //DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA1_Channel4, ENABLE);  //使能USART1 TX DMA1 所指示的通道 
}
void DMA_Stuff(u8 *Data,u8 len)
{
	if(len==0)
	{
		TotalLen=0;
		return;
	}
	u8 i;
	for(i=0;i<len;i++)
	{
		if(TotalLen+i>=SENDBUF_SIZE)
			return;//若发送速率不够快导致缓冲区满则放弃新的数据
		SendBuff[TotalLen+i]=Data[i];
	}
	TotalLen+=len;
}
void DMA1_Channel4_IRQHandler(void)
{
//判断是否为DMA发送完成中断
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4)==SET) 
	{
		DMA_ClearFlag(DMA1_FLAG_TC4);
		DMA_Stuff(0,0);
	}
}
