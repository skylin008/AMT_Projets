#ifndef __LED_H
#define __LED_H
#include "stm32f1xx_hal.h"

typedef enum 
{
  Armed=0,
  Initting=1,
  I2C_ERROR=2,
  Wait_armed=3
  
} LED_State;



void Led_Blink(void);
void check_LED(void);
void set_LED_State(uint8_t nState);

#endif
