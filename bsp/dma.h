#ifndef __DMA_H
#define	__DMA_H	   

#include "stm32f10x.h"

#define SENDBUF_SIZE 1024
void DMA_Config(void);
void DMA_Enable(void);
void DMA_Stuff(u8 *Data,u8 len);
#endif
