#include "app_control.h"
#include "lqr_mode.h"             // LQR_MODE (modo de arranque) y referencias (lo escribe Julia)
#include "lqr_gain_mode0.h"       // Ganancias del modo 0: K.._0 (control_lqr_mode0.jl)
#include "lqr_gain_mode1.h"       // Ganancias del modo 1: K.._1 (control_lqr_mode1.jl)
#include "lqr_gain_mode2.h"       // Ganancias del modo 2: K.._2 (control_lqr_mode2.jl)

#if (LQR_MODE < 0) || (LQR_MODE > 2)
#error "LQR_MODE debe ser 0, 1 o 2 (ver lqr_mode.h)"
#endif

//���״̬���� Related status data
float x_pose=0.0, x_speed, angle_x, gyro_x, angle_z=0, gyro_z, last_angle=0;
float L_accel, R_accel, velocity_L, velocity_R;
float speed_L, speed_R;   // Velocidad medida de cada rueda (m/s)


// Ganancias LQR de todos los modos. Se compilan todas para poder cambiar de modo en
// ejecucion con Set_Control_Mode(); los indices Kij siguen el estado completo
// x = [x, v, pitch, pitch_rate, yaw, yaw_rate] (j = 1..6) en todos los modos.

// Modo 0: u = -K (x - x_ref)                                    (lqr_gain_mode0.h)
float K11_0 = K11_0_LQR, K12_0 = K12_0_LQR, K13_0 = K13_0_LQR, K14_0 = K14_0_LQR, K15_0 = K15_0_LQR, K16_0 = K16_0_LQR;
float K21_0 = K21_0_LQR, K22_0 = K22_0_LQR, K23_0 = K23_0_LQR, K24_0 = K24_0_LQR, K25_0 = K25_0_LQR, K26_0 = K26_0_LQR;

// Modo 1: u = -(K x + Ki z1),  z1 = integral(ref_vel - v),  sin estado de posicion   (lqr_gain_mode1.h)
float K12_1 = K12_1_LQR, K13_1 = K13_1_LQR, K14_1 = K14_1_LQR, K15_1 = K15_1_LQR, K16_1 = K16_1_LQR, Ki1_1 = Ki1_1_LQR;
float K22_1 = K22_1_LQR, K23_1 = K23_1_LQR, K24_1 = K24_1_LQR, K25_1 = K25_1_LQR, K26_1 = K26_1_LQR, Ki2_1 = Ki2_1_LQR;

// Modo 2: u = -(K (x - x_ref) + Ki z2),  x = [x, v, pitch, pitch_rate, yaw_rate],  z2 = integral(ref_wd - yaw_rate)   (lqr_gain_mode2.h)
float K11_2 = K11_2_LQR, K12_2 = K12_2_LQR, K13_2 = K13_2_LQR, K14_2 = K14_2_LQR, K16_2 = K16_2_LQR, Ki1_2 = Ki1_2_LQR;
float K21_2 = K21_2_LQR, K22_2 = K22_2_LQR, K23_2 = K23_2_LQR, K24_2 = K24_2_LQR, K26_2 = K26_2_LQR, Ki2_2 = Ki2_2_LQR;

// Referencias, compartidas por todos los modos (lqr_mode.h)
float ref_pos = REFPOS_LQR;       // Posicion x (m)                  -> modos 0 y 2
float ref_vel = REFVEL_LQR;       // Velocidad lineal (m/s)          -> modo 1
float ref_yaw = REFYAW_LQR;       // Angulo yaw (rad)                -> modos 0 y 1
float ref_wd  = REFWD_LQR;        // Velocidad angular yaw (rad/s)   -> modo 2
float z1 = 0;                     // Integral del error de velocidad (modo 1)
float z2 = 0;                     // Integral del error de velocidad de yaw (modo 2)
bool sel=true;
// Modo activo. Arranca en el ultimo modo programado desde Julia (LQR_MODE).
volatile u8 control_mode = LQR_MODE;
static volatile u8 mode_request = LQR_MODE;

// Pide un cambio de modo (0, 1 o 2). Se aplica al inicio del siguiente periodo
// de control (5 ms), dentro de la interrupcion, para no cambiar a mitad de calculo.
void Set_Control_Mode(u8 new_mode)
{
	if (new_mode <= 2)
		mode_request = new_mode;
}

u8 Get_Control_Mode(void)
{
	return control_mode;
}

//Ŀ��״ֵ̬ Target state value
float Target_angle_x = 0.0349;    // Pitch de equilibrio (rad)

float time=0;
float error_x = 0;

//�ٶȻ����PWMռ�ձȵı���ϵ�� Proportional coefficient for converting speed to PWM duty cycle
float Ratio_accel=2400;	


float battery = 12;//��ʼ״̬�������� 12v 12v The initial state is fully charged 12v
u8 Flag_velocity=2;

void EXTI15_10_IRQHandler(void)
{
	int Encoder_Left,Encoder_Right;             					//���ұ��������������  Pulse counting of left and right encoders
	u8 motor_off;
   

		
  // ����Ƿ����ж��¼�  Check if any interruption events have occurred
  if(MPU6050_INT==0)		
	{   
	    time +=0.005; // aqui llevo el tiempo
		
		EXTI->PR=1<<12;                           					//����жϱ�־λ Clear interrupt flag bit
		
		Get_Angle(GET_Angle_Way);                     			//������̬��5msһ�Σ����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч�� Updating the posture once every 5ms, a higher sampling frequency can improve the effectiveness of Kalman filtering and complementary filtering
		Encoder_Left=Read_Encoder(MOTOR_ID_ML);            					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ�� Read the value of the left wheel encoder, forward is positive, backward is negative
		Encoder_Right=-Read_Encoder(MOTOR_ID_MR);           					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ�� Read the value of the right wheel encoder, forward is positive, backward is negative
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right); //��ȡ�ٶ� Obtain speed
				
		// Velocidad de cada rueda (m/s): cuentas por periodo -> mm -> m (/1000)
		speed_L = Encoder_Left *PI*Diameter_67/1000/1560*Control_Frequency;
		speed_R = Encoder_Right*PI*Diameter_67/1000/1560*Control_Frequency;
		x_speed = (speed_L + speed_R)/2;
		x_pose+=x_speed/Control_Frequency;
		//��ȡ���(rad)�����ٶ�(rad/s)  Obtain inclination angle (rad) and angular velocity (rad/s)
		angle_x=Angle_Balance/180*PI;
		gyro_x=(angle_x-last_angle)*Control_Frequency;
		last_angle=angle_x;
		//��ȡת���ٶ�(rad/s)��ת���(rad)  Obtain steering speed (rad/s) and steering angle (rad)
		// gyro_z en rad/s (Wheel_spacing esta en mm). Positivo = giro a la derecha
		// (horario visto desde arriba): el mismo convenio del modelo en Julia, donde
		// B_61 > 0 (acelerar la rueda izquierda hace crecer el yaw).
		gyro_z=(speed_L - speed_R)/(Wheel_spacing/1000);
		angle_z+= gyro_z/Control_Frequency;
		
		
		// Mientras el control esta apagado (antes de KEY1, robot caido >40 grados
		// o bateria baja) no se acumula posicion ni yaw: al activarse, el robot
		// regula alrededor del punto donde esta y no intenta volver al de encendido.
		motor_off = Turn_Off(Angle_Balance,battery);
		if(motor_off)
		{
			x_pose = 0;
			angle_z = 0;
			z1=0;
			z2=0;
		}

		//�����������(LQR������) Calculate input variables (LQR controller)
        
		// Cambio de modo pedido con Set_Control_Mode(): se reinician la posicion,
		// el yaw y las integrales para que el nuevo modo arranque sin saltos (p. ej. al
		// pasar del modo 2 al 0 el robot no intenta deshacer las vueltas que dio).
		if (mode_request != control_mode)
		{
			control_mode = mode_request;
			ref_pos = x_pose;
			ref_yaw = angle_z;
			z1 = 0;
			z2 = 0;
		}

		if (control_mode == 0)
		{
			

			// Modo 0: realimentacion de estados con referencias de posicion y yaw
			L_accel = - (K11_0*(x_pose-ref_pos) + K12_0*x_speed + K13_0*(angle_x-Target_angle_x) + K14_0*gyro_x + K15_0*(angle_z-ref_yaw) + K16_0*gyro_z);
			R_accel = - (K21_0*(x_pose-ref_pos) + K22_0*x_speed + K23_0*(angle_x-Target_angle_x) + K24_0*gyro_x + K25_0*(angle_z-ref_yaw) + K26_0*gyro_z);
		}
		else if (control_mode == 1)
		{
			// Modo 1: accion integral sobre el error de velocidad lineal

			L_accel = - (Ki1_1*z1 + K12_1*x_speed + K13_1*(angle_x-Target_angle_x) + K14_1*gyro_x + K15_1*(angle_z-ref_yaw) + K16_1*gyro_z);
			R_accel = - (Ki2_1*z1 + K22_1*x_speed + K23_1*(angle_x-Target_angle_x) + K24_1*gyro_x + K25_1*(angle_z-ref_yaw) + K26_1*gyro_z);
			z1 += (ref_vel - x_speed)/Control_Frequency;    // Integral del error de velocidad
		}
		else
		{
			// Modo 2: velocidad angular de yaw con accion integral, manteniendo la posicion
			L_accel = - (K11_2*(x_pose-ref_pos) + K12_2*x_speed + K13_2*(angle_x-Target_angle_x) + K14_2*gyro_x + K16_2*(gyro_z-ref_wd) + Ki1_2*z2);
			R_accel = - (K21_2*(x_pose-ref_pos) + K22_2*x_speed + K23_2*(angle_x-Target_angle_x) + K24_2*gyro_x + K26_2*(gyro_z-ref_wd) + Ki2_2*z2);
			z2 += (ref_wd - gyro_z)/Control_Frequency;    // Integral del error de velocidad de yaw
		}

		// Cada rueda parte de su propia velocidad medida y no del promedio x_speed:
		// asi la diferencia entre ruedas (el giro) se conserva de un periodo al
		// siguiente, y la aceleracion diferencial del LQR integra la velocidad de
		// yaw como en el modelo. El promedio de las dos ruedas no cambia.
		velocity_L=(int)(Ratio_accel*(speed_L+L_accel/Control_Frequency));
		velocity_R=(int)(Ratio_accel*(speed_R+R_accel/Control_Frequency));


		
		//PWM�޷�  PWM limiting
		Motor_Left=PWM_Limit(velocity_L,2600,-2600); //25khz->2592 
		Motor_Right=PWM_Limit(velocity_R,2600,-2600);		
		
		//�˵�����  Filter out dead zones
		Motor_Left = PWM_Ignore(Motor_Left);
		Motor_Right = PWM_Ignore(Motor_Right);

		
		if(motor_off==0)     					//����������쳣 		If there are no abnormalities
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
