#include "IMU.h"
#include <math.h>
//#include "Copter_Param.h"

#define RAD_TO_DEG 57.295779513082320876798154814105  // 弧度转角度的转换率
//卡尔曼参数	
float Q_angle = 0.03;  // 角度数据置信度 
float Q_gyro  = 0.01; // 角速度数据置信度 
float R_angle = 0.1; // 方差噪声
char  C_0 = 1;
float Q_bias_x,Q_bias_y,Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
float Pitch; //X最终倾斜角度
float Roll; //Y最终倾斜角度
float Gyro_P_Final ;
float Gyro_R_Final ;
float Gyro_Y_Final ;


/**
  * @brief  更新IMU
  * @param  None
  * @retval None
  */
void update_IMU(float dt)
{
  float ACC_X,ACC_Y,ACC_Z,Gyro_X,Gyro_Y,Gyro_Z,ACC_Pitch,ACC_Roll;  //暂存数据
  //MPU6050读取原始数据并简单滤波
  get_MPU6050_RawData(&MPU6050);
  ACC_Windows_Filter(&MPU6050);
  //根据设置量程规范化
  ACC_X=(float)MPU6050.ACC.X/8192.0f;
  ACC_Y=(float)MPU6050.ACC.Y/8192.0f;
  ACC_Z=(float)MPU6050.ACC.Z/8192.0f;
  Gyro_X=(float)MPU6050.Gyro.X/32.8f;
  Gyro_Y=(float)MPU6050.Gyro.Y/32.8f;
  Gyro_Z=(float)MPU6050.Gyro.Z/32.8f;
  Gyro_Z*=-1;
  MPU6050.Axis_Rate.Yaw=Gyro_Z;
  //弧度转为角度
  MPU6050.Axis_Angle.Yaw+=Gyro_Z*dt;
  ACC_Pitch=atan2(ACC_Y,ACC_Z) * RAD_TO_DEG;
  ACC_Roll=atan(-ACC_X / sqrt(ACC_Y * ACC_Y + ACC_Z * ACC_Z)) * RAD_TO_DEG;
#if 0
  int16_t Pitch=(float)10*Gyro_X;
  int16_t Roll=(float)10*Gyro_Y;
  int16_t Yaw=(float)10*Gyro_Z;
  int16_t buffer[3];
  buffer[0]=Roll;
  buffer[1]=Pitch;
  buffer[2]=Yaw;
  Wava_Send(buffer,3);
#endif
  //卡尔曼滤波解算角度,滤波角速度
  Kalman_Filter_X(ACC_Pitch,Gyro_X,&MPU6050,dt);
  Kalman_Filter_Y(ACC_Roll,Gyro_Y,&MPU6050,dt);
  //int16_t temp=10*MPU6050.Axis_Rate.Pitch;
  //printf("%d\n",temp);

#if 0    //角度
  int16_t Pitch=(float)100*MPU6050.Axis_Angle.Pitch;
  int16_t Roll=(float)100*MPU6050.Axis_Angle.Roll;
  int16_t Yaw=(float)100*MPU6050.Axis_Angle.Yaw;
  int16_t buffer[3];
  buffer[0]=Roll;
  buffer[1]=Pitch;
  buffer[2]=Yaw;
  Wava_Send(buffer,3);
  //printf("%d\n",(int)Pitch);
#endif
  #if 0            //角速度
  int16_t Pitch=(float)100*MPU6050.Axis_Rate.Pitch;
  int16_t Roll=(float)100*MPU6050.Axis_Rate.Roll;
  int16_t Yaw=(float)100*MPU6050.Axis_Rate.Yaw;
  int16_t buffer[3];
  buffer[0]=Roll;
  buffer[1]=Pitch;
  buffer[2]=Yaw;
  Wava_Send(buffer,3);
  //printf("%d\n",(int)Pitch);
#endif

  


}

/*************************************************
void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数		
功能：陀螺仪数据与加速度计数据通过滤波算法融合
输入参数：
float Accel   加速度计计算的角度
float Gyro   陀螺仪角速度
Pitch  融合后的角度
Gyro_P_Final  融合后的角速度
输出参数：滤波后的角度及角速度
返回值：无
**************************************************/
void Kalman_Filter_X(float Accel,float Gyro,MPU6050_t *MPU6050,float dt) //卡尔曼函数		
{
	(MPU6050->Axis_Angle.Pitch) += (Gyro - Q_bias_x) * dt; //先验估计
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]= Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - (MPU6050->Axis_Angle.Pitch);	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	(MPU6050->Axis_Angle.Pitch) += K_0 * Angle_err;	 //后验估计
	Q_bias_x += K_1 * Angle_err;	 //后验估计
	(MPU6050->Axis_Rate.Pitch)   = Gyro - Q_bias_x;	 //输出值(后验估计)的微分=角速度
}

void Kalman_Filter_Y(float Accel,float Gyro,MPU6050_t *MPU6050,float dt) //卡尔曼函数		
{
	MPU6050->Axis_Angle.Roll += (Gyro - Q_bias_y) * dt; //先验估计
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - MPU6050->Axis_Angle.Roll;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	MPU6050->Axis_Angle.Roll	+= K_0 * Angle_err;	 //后验估计
	Q_bias_y	+= K_1 * Angle_err;	 //后验估计
	MPU6050->Axis_Rate.Roll= Gyro - Q_bias_y;	 //输出值(后验估计)的微分=角速度
}






