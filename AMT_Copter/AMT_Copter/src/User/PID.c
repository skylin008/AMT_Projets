#include "PID.h"



PID_t pidRoll,pidPitch,pidYaw;


/**
  * @brief  PID运算--位置式
  * @param  
  * @retval None
  */
float updatePID(PID_t *PID,float setpoint, float input, float dt)
{
  float error=setpoint-input;   //error=期望-实际
  //P-term
  float pTerm=PID->Kp*error;
  //I-term
  PID->iTerm+=PID->Ki*(error+PID->lastError)/2.0f*dt;
  PID->iTerm=constrain(PID->iTerm,-PID->integrationLimit,PID->integrationLimit);  //积分限幅
  //D-term
  float dTerm=PID->Kd*(error-PID->lastError)/dt;    //微分：低通滤波？

  PID->lastError=error;

  return pTerm+PID->iTerm+dTerm;
}


