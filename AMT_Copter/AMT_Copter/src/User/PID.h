#ifndef __PID_H
#define __PID_H
#include "stm32f1xx_hal.h"

#define constrain(amt,low,high)  ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))  

typedef struct
{
  float Kp,Ki,Kd;            //PID值
  float iTerm;               //I积分值
  float lastError;           //上一次误差
  float integrationLimit;    //积分限幅


}PID_t;


extern PID_t pidRoll,pidPitch,pidYaw;
float updatePID(PID_t *PID,float setpoint, float input, float dt);



#endif
