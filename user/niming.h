#ifndef __NIMING_H
#define __NIMING_H

#include "dma.h"

//ANO_CMD
#define ACC_CALI      0x01
#define GYRO_CALI     0x02
#define MAG_CALI      0x04
#define BARO_CALI     0x08
#define PID_REQUIRE   0x10
#define PID_SENDBACK  0x20
#define RESET_PARAM   0x80

extern u8 ANO_CMD;
void ANO_Send_User_Data(s16 *User_Frame,u8 len,u8 fun);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, float alt, u8 fly_model, u8 Armed);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle);
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_Station_Send_flydata(short thr,short yaw,short rol,short pit,short aux1,short aux2,short aux3,short aux4,short aux5,short aux6);
void ANO_DT_Send_Speed(float x_s,float y_s,float z_s);
u8 ANO_Data_Receive_Prepare(u8 data,u8 *RxBuffer);
u8 ANO_Data_Receive_Anl(u8 *data_buf);
extern u8 PIDReceiveTemp[6][18];
extern short FlyData[10];
//void ANO_Send_MotoPWM(PWM_Moto *motor);

#endif
