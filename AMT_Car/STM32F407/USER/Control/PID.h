#ifndef __PID_H
#define __PID_H
#include "stm32f4xx.h" 


#define INC 0  //����ʽ
#define POS 1 //λ��ʽ


typedef struct
{
  float Kp;
	float Ki;
	float Kd;
	float lastError;  //��һ�ε����
	float prevError;  //���ϴ�������ʽPID��ʹ��
	float iTerm;      //�����ۻ���λ��ʽPID��ʹ��
	float Output;  
	
}PID_typedef;

//void initPIDParam();
void reset_PID(PID_typedef* PID);
void update_PID_INC(PID_typedef* PID,float target,float input);   //����ʽPID
void update_PID_POS(PID_typedef* PID);   //λ��ʽPID


#endif
