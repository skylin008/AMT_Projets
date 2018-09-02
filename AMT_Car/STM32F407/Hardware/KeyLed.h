#ifndef __KEYLED_H
#define __KEYLED_H
#include "stm32f4xx.h" 

#define LED2_ON GPIO_ResetBits(GPIOD,GPIO_Pin_4)
#define LED2_OFF GPIO_SetBits(GPIOD,GPIO_Pin_4)
#define LED3_ON GPIO_ResetBits(GPIOD,GPIO_Pin_7)
#define LED3_OFF GPIO_SetBits(GPIOD,GPIO_Pin_7)
#define LED2 1
#define LED3 2


void init_KeyLed(void);
void Led_Blink(u8 LED);




#endif
