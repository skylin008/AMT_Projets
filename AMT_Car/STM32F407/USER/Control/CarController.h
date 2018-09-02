#ifndef __CARCONTROLLER_H
#define __CARCONTROLLER_H
#include "stm32f4xx.h" 
#include "MotorController.h"

#define Speed_Max 1000   //С������ٶ�



typedef struct
{
  Motor_typedef Motor[4];
  s16 Speed;           //С�����ٶ�
	s16 Pos_x;          //ȫ�ֶ�λx��
	s16 Pos_y;          //ȫ�ֶ�λy��

}Car_typedef;


extern Car_typedef AMT_Car;
void init_Car(void);





#endif
