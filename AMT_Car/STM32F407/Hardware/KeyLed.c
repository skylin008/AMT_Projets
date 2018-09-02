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
	EXTI_InitTypeDef   EXTI_InitStructure;        //�ж���
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��

	/**************LED_GPIO����************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                   //����
  GPIO_Init(GPIOD, &GPIO_InitStructure);                         //��ʼ��GPIO

	LED2_OFF;
	LED3_OFF;
	
	/***************KEY_GPIO����**************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;             //��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //����
  GPIO_Init(GPIOE, &GPIO_InitStructure);                   //��ʼ��
	  //�ⲿ�ж�����
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);//PE2 ���ӵ��ж���2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);//PE3 ���ӵ��ж���2
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line2|EXTI_Line3;         //ѡ���ж���·
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//�ⲿ�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//�ⲿ�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
}

void Led_Blink(u8 LED)   //LED��˸
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




void EXTI2_IRQHandler(void)   //KEY1�ⲿ�ж�
{

	//delay_ms(10);	//����
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0)
	{
	   //Led_Blink(LED2);
		AMT_Car.Speed+=100;
		AMT_Car.Speed=constrain(AMT_Car.Speed,-Speed_Max,Speed_Max);
	
	}	
	EXTI_ClearITPendingBit(EXTI_Line2);//����жϱ�־λ  
}
void EXTI3_IRQHandler(void)   //KEY2�ⲿ�ж�
{
  if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0)
	{
	   //Led_Blink(LED3);
	  AMT_Car.Speed-=100;
	  AMT_Car.Speed=constrain(AMT_Car.Speed,-Speed_Max,Speed_Max);
	}
	EXTI_ClearITPendingBit(EXTI_Line3);//����жϱ�־λ  
}
