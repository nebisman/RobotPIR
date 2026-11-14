#ifndef __APP_CONTROL_H_
#define __APP_CONTROL_H_

#include "AllHeader.h" 

#define MPU6050_INT PAin(12)   //PA12连接到MPU6050的中断引脚  PA12 is connected to the interrupt pin of MPU6050

void Get_Angle(u8 way);
void Set_Control_Mode(u8 new_mode);   // 0: realimentacion de estados, 1: integral de velocidad, 2: velocidad de yaw
u8 Get_Control_Mode(void);

#endif

