#ifndef __STABILIZE_H
#define __STABILIZE_H
#include "stm32f1xx_hal.h"

typedef struct
{
  uint8_t Acro;             //特技模式，无外环
  uint8_t Stabilize;        //自稳模式
  uint8_t Headhold;         //锁头模式，Yaw加外环
  

}Fly_Mode_t;

extern Fly_Mode_t Fly_Mode;

void check_FlyMode(void);
void resetPIDRollPitchYaw(void);
void init_PID(void);

#endif
