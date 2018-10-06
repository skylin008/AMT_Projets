#include "LED.h"
#include "gpio.h"

uint8_t Led_State;
uint16_t ms10count=0;
/**
  * @brief  LED灯语  
  * @param  
  * @retval None
  */
void check_LED(void)
{
    ms10count++;
  switch(Led_State)
  {
    case Armed:        //解锁状态，快闪
    //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0);
    if(ms10count>=10) {ms10count=0;Led_Blink();}
    break;

    case Initting:   //正在初始化，20Hz闪烁
    if(ms10count>=5) {ms10count=0;Led_Blink();}
    break;

    case I2C_ERROR:  //I2C总线故障,快闪500ms，熄灭1s
    if(ms10count<50)
    {
        if(ms10count%5==0)
        Led_Blink();}
    else if(ms10count>=50) {HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1);}
    if(ms10count>150) ms10count=0;
    break;
    case Wait_armed:    //等待解锁信号，600ms慢闪
       if(ms10count>50)
       {
         Led_Blink();
         ms10count=0;
       }
    break;



  }




}

//设置LED状态
void set_LED_State(uint8_t nState)
{
  Led_State=nState;
  ms10count=0;
}
void Led_Blink(void)
{
  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

}

