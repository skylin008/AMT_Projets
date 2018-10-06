#include "Stabilize.h"
#include "Data_Exchange.h"
#include "PID.h"
#include "Copter_Param.h"


Fly_Mode_t Fly_Mode;


void check_FlyMode(void)
{
    Fly_Mode.Headhold=RESET;
  if(Rx_Channel.Rx_AUX1<1250) {Fly_Mode.Acro=SET;Fly_Mode.Stabilize=RESET;}
  else if(Rx_Channel.Rx_AUX1<1750) {Fly_Mode.Stabilize=SET;Fly_Mode.Acro=RESET;}
  else if(Rx_Channel.Rx_AUX1>1750) {Fly_Mode.Stabilize=SET;Fly_Mode.Acro=RESET;Fly_Mode.Headhold=SET;}
}

/**
  * @brief  载入PID参数
  * @param  
  * @retval None
  */
void init_PID(void)
{
  //Yaw角参数
  pidYaw.Kp=pidYaw_Kp;
  pidYaw.Ki=pidYaw_Ki;
  pidYaw.Kd=pidYaw_Kd;
  pidYaw.integrationLimit=iTerm_LimitYaw;
  //Roll，Pitch参数
  pidRoll.Kp=pidPitch.Kp=pidRollPitch_Kp;
  pidRoll.Ki=pidPitch.Ki=pidRollPitch_Ki;
  pidRoll.Kd=pidPitch.Kd=pidRollPitch_Kd;
  pidRoll.integrationLimit=pidPitch.integrationLimit=iTerm_LimitRollPitch;
}
/**
  * @brief  重置PID中间参数
  * @param  
  * @retval None
  */
void resetPIDRollPitchYaw(void)
{
  pidRoll.iTerm=pidRoll.lastError=0.0f;
  pidPitch.iTerm=pidPitch.lastError=0.0f;
  pidYaw.iTerm=pidYaw.lastError=0.0f;
}
