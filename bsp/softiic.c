#include "softiic.h"

//初始化IIC
void IIC_Init(void)
{					     
  GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//先使能外设IO PORTB时钟
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	 // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 

  GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5);						 //PB8,PB9 输出高	
}
 
//IIC延时
void IIC_Delay(void)
{
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
}

//产生IIC起始信号
void IIC_Start(void)
{
	MPU_SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	IIC_Delay();
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	IIC_Delay();
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	MPU_SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	IIC_Delay();
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	IIC_Delay();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
void IIC_Wait_Ack(void)
{
	MPU_SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;IIC_Delay();	   
	IIC_SCL=1;IIC_Delay();	 
	while(MPU_READ_SDA);
	IIC_SCL=0;//时钟输出0 	   
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	MPU_SDA_OUT();
	IIC_SDA=0;
	IIC_Delay();
	IIC_SCL=1;
	IIC_Delay();
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	MPU_SDA_OUT();
	IIC_SDA=1;
	IIC_Delay();
	IIC_SCL=1;
	IIC_Delay();
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		    IIC_SCL=1;
		    IIC_Delay(); 
		    IIC_SCL=0;	
		    IIC_Delay();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        IIC_Delay();
		IIC_SCL=1;
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		IIC_Delay(); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

//IIC写一个字节
//addr:IIR从设备7位地址
//reg:寄存器地址
//data:数据
void IIC_Write_Reg(u8 addr, u8 reg, u8 data)
{
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(reg);	//写寄存器地址
	IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	IIC_Wait_Ack();		//等待应答 
	IIC_Stop();
}
//IIC读一个字节
//addr:IIR从设备7位地址
//reg:寄存器地址
//返回值:读到的数据
u8 IIC_Read_Reg(u8 addr, u8 reg)
{
	u8 res;
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);//发送器件地址+写命令
	IIC_Wait_Ack();		//等待应答
	IIC_Send_Byte(reg);	//写寄存器地址
	IIC_Wait_Ack();		//等待应答
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 1);//发送器件地址+读命令
	IIC_Wait_Ack();		//等待应答
	res = IIC_Read_Byte(0);//读取数据,发送nACK
	IIC_Stop();
	return res;
}
