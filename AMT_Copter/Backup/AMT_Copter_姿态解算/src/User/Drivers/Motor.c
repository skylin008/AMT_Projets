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
   TIM1->CCR1=0;
   TIM1->CCR2=0;
   TIM1->CCR3=0;
   TIM1->CCR4=0;
}
