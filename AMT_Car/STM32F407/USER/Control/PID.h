#ifndef __PID_H
#define __PID_H
#include "stm32f4xx.h" 


#define INC 0  //增量式
#define POS 1 //位置式


typedef struct
{
  float Kp;
	float Ki;
	float Kd;
	float lastError;  //上一次的误差
	float prevError;  //上上次误差，增量式PID中使用
	float iTerm;      //积分累积，位置式PID中使用
	float Output;  
	
}PID_typedef;

//void initPIDParam();
void reset_PID(PID_typedef* PID);
void update_PID_INC(PID_typedef* PID,float target,float input);   //增量式PID
void update_PID_POS(PID_typedef* PID);   //位置式PID


#endif
