#include "app_control.h"
#include "lqr_gain.h"

//���״̬���� Related status data
float x_pose=0, x_speed, angle_x, gyro_x, angle_z=0, gyro_z, last_angle=0;
float L_accel, R_accel, velocity_L, velocity_R;

//LQR״̬����ϵ�� LQR state feedback coefficient
float K11 = K11_LQR, K12 = K12_LQR, K13 = K13_LQR, K14 = K14_LQR, K15 = K15_LQR, K16 = K16_LQR;
float K21 = K21_LQR, K22 = K22_LQR, K23 = K23_LQR, K24 = K24_LQR, K25 = K25_LQR, K26 = K26_LQR;
float K_pos = Kr_pos;
float ref_pos = 0.05;

// Ganancias de yaw originales (rueda izq: K15,K16; der: K25,K26)
float K15OLD = K15_LQR, K16OLD = K16_LQR, K25OLD = K25_LQR, K26OLD = K26_LQR;

//Ŀ��״ֵ̬ Target state value
float Target_x_speed=0, Target_angle_x=0.0, Target_gyro_z=0;

//�ٶȻ����PWMռ�ձȵı���ϵ�� Proportional coefficient for converting speed to PWM duty cycle
float Ratio_accel=2400;	


float battery = 12;//��ʼ״̬�������� 12v 12v The initial state is fully charged 12v
u8 Flag_velocity=2;

void EXTI15_10_IRQHandler(void)
{
	int Encoder_Left,Encoder_Right;             					//���ұ��������������  Pulse counting of left and right encoders
		
  // ����Ƿ����ж��¼�  Check if any interruption events have occurred
  if(MPU6050_INT==0)		
	{   
		EXTI->PR=1<<12;                           					//����жϱ�־λ Clear interrupt flag bit
		
		Get_Angle(GET_Angle_Way);                     			//������̬��5msһ�Σ����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч�� Updating the posture once every 5ms, a higher sampling frequency can improve the effectiveness of Kalman filtering and complementary filtering
		Encoder_Left=Read_Encoder(MOTOR_ID_ML);            					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ�� Read the value of the left wheel encoder, forward is positive, backward is negative
		Encoder_Right=-Read_Encoder(MOTOR_ID_MR);           					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ�� Read the value of the right wheel encoder, forward is positive, backward is negative
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right); //��ȡ�ٶ� Obtain speed
				
		x_speed=(Encoder_Left+Encoder_Right)/2*PI*Diameter_67/1000/1560*Control_Frequency; // ��1000��mmתm  Except for 1000 mm to m
		x_pose+=x_speed/Control_Frequency;
		//��ȡ���(rad)�����ٶ�(rad/s)  Obtain inclination angle (rad) and angular velocity (rad/s)
		angle_x=Angle_Balance/180*PI;
		gyro_x=(angle_x-last_angle)*Control_Frequency;
		last_angle=angle_x;
		//��ȡת���ٶ�(rad/s)��ת���(rad)  Obtain steering speed (rad/s) and steering angle (rad)
		gyro_z=(Encoder_Right-Encoder_Left)/Wheel_spacing/1000*PI*Diameter_67/1000/1560*Control_Frequency;
		angle_z+=gyro_z/Control_Frequency;
				
		if(g_newcarstate==enRUN)  
		{
//			Target_x_speed = 0.6f;
//			Target_x_speed = Target_x_speed/Flag_velocity;//ǰ���ٶ�(m/s)  Forward speed (m/s)
			Target_x_speed = 0.5f;
			x_pose = 0;
		}			
		else if(g_newcarstate==enBACK)
		{
			Target_x_speed = -0.6f;
			Target_x_speed = Target_x_speed/Flag_velocity;//�����ٶ�(m/s)   Backward speed (m/s)
			x_pose = 0;
		}	
		//����С����ת����ת  Control the left and right turns of the car
		else if(g_newcarstate==enLEFT)
		{
			Target_gyro_z = -4;					//��תת���ٶ�(rad/s)  Left turn speed (rad/s)
			Target_gyro_z = Target_gyro_z/Flag_velocity;
			angle_z = 0;
		}
		else if(g_newcarstate==enRIGHT)
		{
			Target_gyro_z = 4;					//��תת���ٶ�(rad/s)  Right turn speed (rad/s)
			Target_gyro_z = Target_gyro_z/Flag_velocity;
			angle_z = 0;
		}
		else if(g_newcarstate == enTLEFT)//���� Left turn
		{
			Target_gyro_z = -4;					//��תת���ٶ�(rad/s)  Left turn speed (rad/s)
			Target_gyro_z = Target_gyro_z/Flag_velocity;
			angle_z = 0;
			K15 =  22.3607f; K25 = -22.3607f;
			K16 =  22.3607f; K26 = -22.3607f;
		}
		else if(g_newcarstate == enTRIGHT)//����  Right rotation
		{
			Target_gyro_z = 4;					//��תת���ٶ�(rad/s)  Right turn speed (rad/s)
			Target_gyro_z = Target_gyro_z/Flag_velocity;
			angle_z = 0;
			K15 =  22.3607f; K25 = -22.3607f;
			K16 =  22.3607f; K26 = -22.3607f;
			
		}
		//���������ܣ����棩  Ultrasonic function (follow)
	  else if(g_newcarstate==enFollow) //���� follow
		{
			Target_x_speed = 0.5;				//ǰ���ٶ�(m/s)  Forward speed (m/s)
			x_pose = 0;
			K15 = 0; K25 = 0;
			K16 = 0; K26 = 0;
		}
		else if(g_newcarstate==enAvoid) //���  avoid
		{
			Target_x_speed = -0.3;				//�����ٶ�(m/s) Backward speed (m/s)
			x_pose = 0;
			K15 = 0; K25 = 0;
			K16 = 0; K26 = 0;
		}
		else
		{
			
			Target_x_speed = 0;				//ƽ���ٶ�(m/s) Balance speed (m/s)
			Target_gyro_z = 0;						//ƽ��ת���ٶ�(rad/s) Balanced steering speed (rad/s)
			K15 = K15OLD; K25 = K25OLD;
			K16 = K16OLD; K26 = K26OLD;
		}
		
		//�����������(LQR������) Calculate input variables (LQR controller)
		L_accel= K_pos*ref_pos -(K11*x_pose+K12*(x_speed-Target_x_speed)+K13*(angle_x-Target_angle_x)+K14*gyro_x+K15*angle_z+K16*(gyro_z-Target_gyro_z));
		R_accel= K_pos*ref_pos -(K21*x_pose+K22*(x_speed-Target_x_speed)+K23*(angle_x-Target_angle_x)+K24*gyro_x+K25*angle_z+K26*(gyro_z-Target_gyro_z));
		//�ٶȻ����PWMռ�ձ�  Convert speed to PWM duty cycle
		velocity_L=(int)(Ratio_accel*(x_speed+L_accel/Control_Frequency));
		velocity_R=(int)(Ratio_accel*(x_speed+R_accel/Control_Frequency));
		
		
		//PWM�޷�  PWM limiting
		Motor_Left=PWM_Limit(velocity_L,2600,-2600); //25khz->2592 
		Motor_Right=PWM_Limit(velocity_R,2600,-2600);		
		
		//�˵�����  Filter out dead zones
		Motor_Left = PWM_Ignore(Motor_Left);
		Motor_Right = PWM_Ignore(Motor_Right);

		
		if(Turn_Off(Angle_Balance,battery)==0)     					//����������쳣 		If there are no abnormalities
			Set_Pwm(Motor_Left,Motor_Right);         					//��ֵ��PWM�Ĵ��� 	Assign to PWM register

   }
	
}

/**************************************************************************
Function: Get angle
Input   : way��The algorithm of getting angle 1��DMP  2��kalman  3��Complementary filtering
Output  : none
�������ܣ���ȡ�Ƕ�	
��ڲ�����way����ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	float gyro_x,gyro_y,accel_x,accel_y,accel_z;
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	Temperature=Read_Temperature();      //��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ� //Read the data from the MPU6050 built-in temperature sensor, which approximately represents the motherboard temperature.
	if(way==1)                           //DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��  //The reading of DMP is interrupted during data collection, strictly following the timing requirements
	{	
		Read_DMP();                      	 //��ȡ���ٶȡ����ٶȡ����  //Read acceleration, angular velocity, and tilt angle
		Angle_Balance=Pitch;             	 //����ƽ�����,ǰ��Ϊ��������Ϊ�� //Update the balance tilt angle, with positive forward tilt and negative backward tilt
		Gyro_Balance=gyro[0];              //����ƽ����ٶ�,ǰ��Ϊ��������Ϊ��  //Update the balance angular velocity, with positive forward tilt and negative backward tilt
		Gyro_Turn=gyro[2];                 //����ת����ٶ� //Update steering angular velocity
		Acceleration_Z=accel[2];           //����Z����ٶȼ� //Update Z-axis accelerometer
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX�������� //Read X-axis gyroscope
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY�������� //Read Y-axis gyroscope
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ�������� //Read Z-axis gyroscope
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ� //Read X-axis accelerometer
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ� //Read Y-axis accelerometer
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ� //Read Z-axis accelerometer
		if(Gyro_X>32768)  Gyro_X-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת�� Data type conversion can also be enforced through short type conversion
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת�� Data type conversion can also be enforced through short type conversion
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //��������ת�� Data type conversion
		if(Accel_X>32768) Accel_X-=65536;                //��������ת�� Data type conversion
		if(Accel_Y>32768) Accel_Y-=65536;                //��������ת�� Data type conversion
		if(Accel_Z>32768) Accel_Z-=65536;                //��������ת�� Data type conversion
		Gyro_Balance=-Gyro_X;                            //����ƽ����ٶ� Update balance angular velocity
		accel_x=Accel_X/1671.84;
		accel_y=Accel_Y/1671.84;
		accel_z=Accel_Z/1671.84;
		gyro_x=Gyro_X/939.8;                              //����������ת�� Gyroscope range conversion
		gyro_y=Gyro_Y/939.8;                              //����������ת�� Gyroscope range conversion
		if(GET_Angle_Way==2)		  	
		{
			 Pitch= KF_X(accel_y,accel_z,-gyro_x)/PI*180;//�������˲� Kalman filtering 
			 Roll = KF_Y(accel_x,accel_z,gyro_y)/PI*180;
		}
		else if(GET_Angle_Way==3) 
		{  
				Accel_Angle_x = atan2(Accel_Y,Accel_Z)*180/PI; //��Accel_Y��accel_y�Ĳ����ó��ĽǶ���һ���ģ�ֻ�Ǳ߳���ͬ The angle obtained using Accel_Y and its parameters is the same, only the side length is different
				Accel_Angle_y = atan2(Accel_X,Accel_Z)*180/PI;
			
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,Gyro_X/16.4);//�����˲� Complementary filtering
			 Roll = -Complementary_Filter_y(Accel_Angle_y,Gyro_Y/16.4);
		}
		Angle_Balance=Pitch;                              //����ƽ�����    Update the balance tilt angle
		Gyro_Turn=Gyro_Z;                                 //����ת����ٶ�  Update steering angular velocity
		Acceleration_Z=Accel_Z;                           //����Z����ٶȼ� Update Z-axis accelerometer
	}

}
