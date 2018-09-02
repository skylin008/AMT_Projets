#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H
#include "stm32f4xx.h" 
#include "MotorController.h"


#define Run 1
#define Stop 0
#define Release 2    //����
#define PWM_Max 10000
#define PWM_Min 0
#define constrain(amt,min,max) ((amt)<=(min)?(min):((amt)>=(max)?(max):(amt))) 


void Motor_Init(void);
void update_SingleMotorState(u8 Motor,u8 State);   //���µ��������״̬
void update_AllMotorState(void);   //�������е����״̬

void Run_AllMotor(void);
void Stop_AllMotor(void);
void Release_AllMotor(void);

void update_PWM(void);

#endif
