#ifndef __DELAY_H
#define __DELAY_H 			   
  
#include "stm32f4xx.h" 	 
void init_delay(u16 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);


#endif





























