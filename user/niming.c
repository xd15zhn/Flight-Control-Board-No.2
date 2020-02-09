#include "niming.h"
/**************文件说明**********************
将数据通过串口发送到电脑上的地面站
ANO_Send_User_Data
ANO_DT_Send_Status
ANO_DT_Send_Senser
ANO_DT_Send_RCData
ANO_DT_Send_PID
********************************************/
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define FRAME_LEN 35//加上帧头等5个字节
#define USER_LEN 30//用户自定义帧最多30个数据
u8 data_to_send[30];
u8 ANO_CMD=BARO_CALI;//上位机到飞控的命令集合
u8 PIDReceiveTemp[6][18];
short FlyData[10]={0,0,0,0,0,0,0,0,0,0};
/**
//function:发送用户自定义帧
*@User_Frame:用户数据数组
*@len:数组长度(30以内)
*@fun:功能码(0xF1~0xFA)
*/
void ANO_Send_User_Data(s16 *User_Frame,u8 len,u8 fun)
{
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=fun;
	data_to_send[_cnt++]=0;
	for(u8 i=0;i<len;i++)
	{
		_temp = (short)User_Frame[i];
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
	}
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	DMA_Stuff(data_to_send,_cnt);
}
/**
//function:发送飞控状态
*@flymode:0,未知；1，姿态；2，定高；3，定点
*/
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, float alt, u8 fly_model, u8 Armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt*100;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (short)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = Armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	DMA_Stuff(data_to_send,_cnt);
}
/**
//function:发送传感器数据
*/
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = 0;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	DMA_Stuff(data_to_send,_cnt);
}
/**
//function:发送遥控器数据
//note:每个通道取值1000~2000
*/
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	DMA_Stuff(data_to_send,_cnt);
}
/**
//function:发送北斗/GPS位置
*@state:0，
*/
void ANO_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x04;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=state;
	data_to_send[_cnt++]=sat_num;
	
	_temp2 = lon;//经度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	_temp2 = lat;//纬度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	
	_temp = (s16)(100 *back_home_angle);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	DMA_Stuff(data_to_send,_cnt);
}
/**
//fun:功能字,6组PID数据对应0x10~0x15
//note:发送PID数据,一组3个PID9个数据,共6组,该函数只发一组
*/
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	DMA_Stuff(data_to_send,_cnt);
}
/**
//function:该函数本来应该是地面站发送控制数据给飞控，这里可用于双机通信时主机发送控制数据给从机
*/
void ANO_Station_Send_flydata(short thr,short yaw,short rol,short pit,short aux1,short aux2,short aux3,short aux4,short aux5,short aux6)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	
	_temp = (short)(thr*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(aux1*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(aux2*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(aux3*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(aux4*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(aux5*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (short)(aux6*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	DMA_Stuff(data_to_send,_cnt);
}
/**
//function:发送3轴速度
*/
void ANO_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	DMA_Stuff(data_to_send,_cnt);
}
/**
*@function:接收到上位机发出的数据后发送校验数据
*@head:功能字
*@check_sum:接收数据的最后一位，也就是校验位
*/
void ANO_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;

	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;
	
	for(u8 i=0;i<7;i++)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET){};
			USART_SendData(USART1,data_to_send[i]);
	}
}
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//解析出符合格式的数据帧后剥离帧头0xAA,0xAF
u8 ANO_Data_Receive_Prepare(u8 data,u8 *RxBuffer)
{
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;

	switch (state)
	{
		case 0:
			if(data==0xAA)state=1;
			break;
		case 1:
			if(data==0xAF)state=2;
			break;
		case 2:
			if(data<0xF1)
			{
				RxBuffer[0]=data;//功能字
				state=3;
			}
			break;
		case 3:
			if(data<50)
			{
				RxBuffer[1]=data;//数据帧长度
				_data_len=data;
				_data_cnt=0;
				state=4;
			}
			break;
		case 4:
			if(_data_len>0)
			{
				RxBuffer[2+_data_cnt++]=data;//有效数据
			}
			if(_data_cnt==_data_len)
				state=5;
			break;
		case 5:
			state=0;
			RxBuffer[2+_data_cnt]=data;//校验位
			return 0;//数据帧接收完毕
//			break;
		default:break;
	}
	return 1;//未接收完或接收失败
}
u8 ANO_Data_Receive_Anl(u8 *data_buf)
{
	u8 sum=0x59;//0xAA+0xAF
	ANO_CMD=0;//所有标志位清零
	for(u8 i=0;i<(data_buf[1]+2);i++)
		sum +=data_buf[i];
	if(sum!=data_buf[data_buf[1]+2])
		return 1;//校验失败
	switch(data_buf[0])
	{
		case 0x01://命令集合1
			switch(data_buf[2])
			{
				case 0x01:ANO_CMD|=ACC_CALI;
					break;
				case 0x02:ANO_CMD|=GYRO_CALI;
					break;
				case 0x04:ANO_CMD|=MAG_CALI;
					break;
				default:break;
			}
			break;
		case 0x02://命令集合2
			switch(data_buf[2])
			{
				case 0x01:ANO_CMD|=PID_REQUIRE;
					break;
				case 0xA1:ANO_CMD|=RESET_PARAM;
					break;
				default:break;
			}
		case 0x03://飞行控制数据
			FlyData[0]=(data_buf[2]<<8)|data_buf[3];//THR
			FlyData[1]=(data_buf[4]<<8)|data_buf[5];//YAW
			FlyData[2]=(data_buf[6]<<8)|data_buf[7];//ROL
			FlyData[3]=(data_buf[8]<<8)|data_buf[9];//PIT
			FlyData[4]=(data_buf[10]<<8)|data_buf[11];//AUX1
			FlyData[5]=(data_buf[12]<<8)|data_buf[13];//AUX2
			FlyData[6]=(data_buf[14]<<8)|data_buf[15];//AUX3
			FlyData[7]=(data_buf[16]<<8)|data_buf[17];//AUX4
			FlyData[8]=(data_buf[18]<<8)|data_buf[19];//AUX5
			FlyData[9]=(data_buf[20]<<8)|data_buf[21];//AUX6
		break;
		case 0x10://PID数据帧1
			ANO_Send_Check(0x10,sum);
			for(u8 i=0;i<18;i++)
				PIDReceiveTemp[0][i]=data_buf[i+2];
			break;
		case 0x11://PID数据帧2
			ANO_Send_Check(0x11,sum);
			for(u8 i=0;i<18;i++)
				PIDReceiveTemp[1][i]=data_buf[i+2];
			break;
		case 0x12://PID数据帧3
			ANO_Send_Check(0x12,sum);
			for(u8 i=0;i<18;i++)
				PIDReceiveTemp[2][i]=data_buf[i+2];
			break;
		case 0x13://PID数据帧4
			ANO_Send_Check(0x13,sum);
			for(u8 i=0;i<18;i++)
				PIDReceiveTemp[3][i]=data_buf[i+2];
			break;
		case 0x14://PID数据帧5
			ANO_Send_Check(0x14,sum);
			for(u8 i=0;i<18;i++)
				PIDReceiveTemp[4][i]=data_buf[i+2];
			break;
		case 0x15://PID数据帧6
			ANO_Send_Check(0x15,sum);
			for(u8 i=0;i<18;i++)
				PIDReceiveTemp[5][i]=data_buf[i+2];
			ANO_CMD|=PID_SENDBACK;//所有数据发完后再将标志位置1
			break;
		default:break;
	}
	return 0;//校验完成,命令集合相应标志位置1
}
