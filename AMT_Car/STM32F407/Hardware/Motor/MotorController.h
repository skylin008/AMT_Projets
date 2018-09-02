#ifndef __MOTORCONTROLLER_H
#define __MOTORCONTROLLER_H
#include "stm32f4xx.h" 
#include "PID.h"



#define DT 0.0025 //400HZ,2.5ms
typedef struct{
  s16 Encoder_Overflow_CNT;   //����������Ĵ���   ��Χ��-32767~32767     
	s32 Encoder_CNT_Cur;         //�����������ܼ���  //MAX=32767*65535/1320*58*3.14/100=2962741(m)
	s32 Encoder_CNT_Last;        //��һ�α����������ܼ���
	s16 SpeedSet;
	s16 SpeedCur;	              //�����ٶ�MAX=3.14*58*320/60=971 (mm/s)
	s16 Motor_PWM;              //�洢�����ǰ��PWMֵ
	u8 State;        //�洢���������״ֵ̬���ֱ�ΪRun,Stop,Release
  //
	PID_typedef pid;

}Motor_typedef;

//extern Motor_typedef Motor[4];


void init_Motor(void);
void init_MotorController(void);  //��ʼ��
void Motor_SpeedTunner(void);
void reset_MotorController(void); //����PID�м����
void MotorController_Suit(void);  //���������С������





#endif


