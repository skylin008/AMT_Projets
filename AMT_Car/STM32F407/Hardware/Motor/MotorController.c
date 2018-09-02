#include "MotorController.h"
#include "MotorDriver.h"
#include "PID_Param.h"
#include "Encoder.h"
#include "CarController.h"
#include "BTModule.h"


//Motor_typedef Motor[4];

void init_Motor(void)  //��ʼ�����
{  
  Motor_Init();        //������ʼ��
	Stop_AllMotor();
  init_Encoder();         //��ʼ��������
	init_MotorController();   //��ʼ��PID
}

void Motor_SpeedTunner(void)  //PID����
{
	u8 Motor_State=Stop;
	
  //���µ�ǰ�ٶ�
	update_MotorSpeed();
	//���������С�������һ
	MotorController_Suit();
  //PID����
	for(u8 i=0;i<4;i++)
		  if(AMT_Car.Motor[i].State==Run)
		      {					
						Motor_State=Run;
	           update_PID_INC(&AMT_Car.Motor[i].pid,AMT_Car.Motor[i].SpeedSet,AMT_Car.Motor[i].SpeedCur);				
	             AMT_Car.Motor[i].Motor_PWM=AMT_Car.Motor[i].pid.Output=constrain(AMT_Car.Motor[i].pid.Output,PWM_Min,PWM_Max-1); //
		      }
  //PWM���µ����
		if(Motor_State==Run) 
		{
			update_PWM();
		#if 0
       printf("%d %d %d\r\n",AMT_Car.Motor[0].SpeedSet,AMT_Car.Motor[0].SpeedCur,AMT_Car.Motor[0].Motor_PWM);					
#endif
		
		} 
}

void init_MotorController(void)  // ����ʼ��ʱ����
{
	for(u8 i=0;i<4;i++)  //AMT_Car
	{
	  AMT_Car.Motor[i].pid.Kp=MotorController_Kp;
	  AMT_Car.Motor[i].pid.Ki=MotorController_Ki;
		AMT_Car.Motor[i].pid.Kd=MotorController_Kd;
		AMT_Car.Motor[i].SpeedSet=0;
	}
	reset_MotorController();
	
}
void reset_MotorController(void)  //����PID�м����
{
  for(u8 i=0;i<4;i++)
	{
	  AMT_Car.Motor[i].pid.lastError=0.0f;
		AMT_Car.Motor[i].pid.prevError=0.0f;
		AMT_Car.Motor[i].Motor_PWM=AMT_Car.Motor[i].pid.Output=PWM_Max/2;  		
	}
}

void MotorController_Suit(void)  //���������С������
{
	for(u8 i=0;i<4;i++) AMT_Car.Motor[i].SpeedSet=AMT_Car.Speed;
	
  AMT_Car.Motor[0].SpeedSet*=-1;
	AMT_Car.Motor[3].SpeedSet*=-1;
	
}
