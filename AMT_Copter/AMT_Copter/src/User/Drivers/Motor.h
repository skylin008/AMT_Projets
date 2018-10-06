#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f1xx_hal.h"



#define MAXMOTOR          499


void init_Motor(void);
void stop_Motor(void);
void update_Motor(float *Motor);


#endif
