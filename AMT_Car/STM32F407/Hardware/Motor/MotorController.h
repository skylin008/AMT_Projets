#ifndef __MOTORCONTROLLER_H
#define __MOTORCONTROLLER_H
#include "stm32f4xx.h" 
#include "PID.h"



#define DT 0.0025 //400HZ,2.5ms
typedef struct{
  s16 Encoder_Overflow_CNT;   //计数器溢出的次数   范围：-32767~32767     
	s32 Encoder_CNT_Cur;         //编码器脉冲总计数  //MAX=32767*65535/1320*58*3.14/100=2962741(m)
	s32 Encoder_CNT_Last;        //上一次编码器脉冲总计数
	s16 SpeedSet;
	s16 SpeedCur;	              //理论速度MAX=3.14*58*320/60=971 (mm/s)
	s16 Motor_PWM;              //存储电机当前的PWM值
	u8 State;        //存储电机的三个状态值，分别为Run,Stop,Release
  //
	PID_typedef pid;

}Motor_typedef;

//extern Motor_typedef Motor[4];


void init_Motor(void);
void init_MotorController(void);  //初始化
void Motor_SpeedTunner(void);
void reset_MotorController(void); //重置PID中间参数
void MotorController_Suit(void);  //电机方向与小车方向





#endif


