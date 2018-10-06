#include "Headhold.h"
#include "MPU6050.h"
#include "Copter_Param.h"

__IO float Yaw_Hold;        //存储更新的期望Yaw角


/**
  * @brief  Yaw角外环
  * @param  
  * @retval None
  */
float update_Headhold()
{
  float error=Yaw_Hold-MPU6050.Axis_Angle.Yaw;  //error=期望值-实际值
  if(error<-180.0f) error+=360.0f;
  else if(error>180.0f) error-=360.0f;

  return error*headKp;
}

/**
  * @brief  更新Yaw角期望角
  * @param  
  * @retval None
  */
void reset_Headhold()
{
   Yaw_Hold=MPU6050.Axis_Angle.Yaw;
}
