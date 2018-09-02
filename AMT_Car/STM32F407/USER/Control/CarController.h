#ifndef __CARCONTROLLER_H
#define __CARCONTROLLER_H
#include "stm32f4xx.h" 
#include "MotorController.h"

#define Speed_Max 1000   //小车最大速度



typedef struct
{
  Motor_typedef Motor[4];
  s16 Speed;           //小车的速度
	s16 Pos_x;          //全局定位x轴
	s16 Pos_y;          //全局定位y轴

}Car_typedef;


extern Car_typedef AMT_Car;
void init_Car(void);





#endif
