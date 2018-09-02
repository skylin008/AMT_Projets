#include "KeyLed.h"
#include "delay.h" 
#include "MotorDriver.h"
#include "CarController.h"

/*
  KEY1-PE2
	KEY2-PE3
	LED2-PD4
	LED3-PD7
*/
void init_KeyLed(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;        //中断线
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIO时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟

	/**************LED_GPIO配置************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                   //上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);                         //初始化GPIO

	LED2_OFF;
	LED3_OFF;
	
	/***************KEY_GPIO配置**************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;             //普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);                   //初始化
	  //外部中断配置
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);//PE2 连接到中断线2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);//PE3 连接到中断线2
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line2|EXTI_Line3;         //选择中断线路
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//外部中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
}

void Led_Blink(u8 LED)   //LED闪烁
{
  switch(LED)
	{
	  case LED2:			
			  if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4)==0) LED2_OFF;
		    else LED2_ON;		  
		break;		
	  case LED3:
			  if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_7)==0) LED3_OFF;
		    else LED3_ON;		  
		break;
	}
}




void EXTI2_IRQHandler(void)   //KEY1外部中断
{

	//delay_ms(10);	//消抖
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0)
	{
	   //Led_Blink(LED2);
		AMT_Car.Speed+=100;
		AMT_Car.Speed=constrain(AMT_Car.Speed,-Speed_Max,Speed_Max);
	
	}	
	EXTI_ClearITPendingBit(EXTI_Line2);//清除中断标志位  
}
void EXTI3_IRQHandler(void)   //KEY2外部中断
{
  if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0)
	{
	   //Led_Blink(LED3);
	  AMT_Car.Speed-=100;
	  AMT_Car.Speed=constrain(AMT_Car.Speed,-Speed_Max,Speed_Max);
	}
	EXTI_ClearITPendingBit(EXTI_Line3);//清除中断标志位  
}
