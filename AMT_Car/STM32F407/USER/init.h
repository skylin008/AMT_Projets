#ifndef __INIT_H
#define __INIT_H

//Í·ÎÄ¼þ
#include "stm32f4xx.h" 
#include <stdio.h>
#include <string.h>
#include "misc.h"
#include "delay.h" 
#include "KeyLed.h"
#include "BTModule.h"
#include "MotorDriver.h"
#include "Encoder.h"
#include "PID.h"
#include "MotorController.h"
#include "CarController.h"





/******************************/
u8 init_All(void);
void init_TIM6(void);



#endif


