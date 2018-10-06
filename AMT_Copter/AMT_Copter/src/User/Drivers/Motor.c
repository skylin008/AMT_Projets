#include "Motor.h"
#include "tim.h"






/**
  * @brief  电机初始化
  * @param  None
  * @retval None
  */
void init_Motor(void)
{
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  //HAL_TIM_Base_Start_IT(&htim1);                         //打开基本定时器中断更新
   stop_Motor();
}
void stop_Motor(void)
{
  TIM1->CCR1=0;
  TIM1->CCR2=0;
  TIM1->CCR3=0;
  TIM1->CCR4=0;
}
void update_Motor(float *Motor)
{
  float maxMotor=Motor[0];
  for(uint8_t i=0;i<4;i++)
  {
    if(Motor[i]>maxMotor) maxMotor=Motor[i];
  }

  if(maxMotor>MAXMOTOR){
    for(uint8_t i=0;i<4;i++)
      Motor[i]-=maxMotor-MAXMOTOR;
  }

  TIM1->CCR1=(uint16_t)Motor[0];
  TIM1->CCR2=(uint16_t)Motor[1];
  TIM1->CCR3=(uint16_t)Motor[2];
  TIM1->CCR4=(uint16_t)Motor[3];
}
