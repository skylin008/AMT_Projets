#include "init.h"
u8 us500_count=0,Loop_400Hz_flag=0,Loop_200Hz_flag=0,Loop_100Hz_flag=0,Loop_20Hz_flag=0;



u8 init_All(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  init_delay(168);        //时钟初始化
	init_KeyLed();            //按键和LED指示灯初始化  init_KeyLed
	init_USART2();
  init_Car();
	init_TIM6();
	
	
	
	Run_AllMotor();
  printf("Init Success\r\n");
  return 1;
}



void init_TIM6(void) //TIM6作为主循环的时基，500us
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrecture;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	TIM_TimeBaseInitStrecture.TIM_Prescaler = 83;    //(周期)T=((arr+1)*(psc+1))/Tclk(MHZ)     (us)  
  TIM_TimeBaseInitStrecture.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
  TIM_TimeBaseInitStrecture.TIM_Period = 499;/*设置自动重装*/
  TIM_TimeBaseInitStrecture.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStrecture.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStrecture);
	
		/* Enable the TIM6 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM6,ENABLE);
}


void TIM6_DAC_IRQHandler(void)  //500us中断
{
	
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET)
	{
		 //清除中断标志
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update); 
		
		us500_count++;
	  if(us500_count%5==0) Loop_400Hz_flag=1;	 //2.5ms
		if(us500_count%10==0) Loop_200Hz_flag=1;	 //5ms
		if(us500_count%20==0) Loop_100Hz_flag=1;	 //10ms
		if(us500_count>=100) //50ms
		{    
			Led_Blink(LED2);
		  Loop_20Hz_flag=1;
			us500_count=0;
		
		}
	}
}
