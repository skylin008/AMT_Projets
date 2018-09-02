#include "MotorDriver.h"
#include "CarController.h"


/*
  MotorA: IN1-PD14,IN2-PA11(TIM1_CH4)
  MotorB: IN1-PD10,IN2-PA10(TIM1_CH3)
  MotorC: IN1-PC8,IN2-PA9(TIM1_CH2)
  MotorD: IN1-PD8,IN2-PA8(TIM1_CH1)
*/
/*
IN1-0 IN2-0 : OUT1-1 OUT2-0
IN1-0 IN2-1 : OUT1-0 OUT2-1
IN1-1 IN2-0 : OUT1-0 OUT2-0
IN1-1 IN2-1 : OUT1-Z OUT2-Z
*/
void Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrecture;
 TIM_OCInitTypeDef TIM_OCInitStructure;
	
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);/*使能GPIOA的时钟*/
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);/*使能定时器1的时钟*/
/****************GPIO配置***************/
	//IN1_GPIO配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                   //上拉
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_14 ;	
  GPIO_Init(GPIOD, &GPIO_InitStructure);                         //初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
  GPIO_Init(GPIOC, &GPIO_InitStructure);       
	
	//IN2_GPIO配置
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 ;	
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/*复用*/
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/*推挽输出*/
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;/**/
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/**/

GPIO_Init(GPIOA,&GPIO_InitStructure);/*初始化IO*/

GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);    
GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);    
GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);    
GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1);    

TIM_TimeBaseInitStrecture.TIM_Prescaler = 0;    //(频率)f=Tclk/((arr+1)*(psc+1))  //此处16.8KHZ
TIM_TimeBaseInitStrecture.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
TIM_TimeBaseInitStrecture.TIM_Period = 9999;/*设置自动重装*/
TIM_TimeBaseInitStrecture.TIM_ClockDivision = TIM_CKD_DIV1;/**/
TIM_TimeBaseInitStrecture.TIM_RepetitionCounter = 0;/**/
TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStrecture);/*初始化*/

/************************PWM模式配置************************/
TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*设置PWM模式*/
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//输出使能
TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;    //缺这一句TIM1-CH1~3输出有问题
TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;  //TIM1空闲输出
TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Reset;


TIM_OCInitStructure.TIM_Pulse = 5000;/*设置比较寄存器*/
TIM_OC1Init(TIM1,&TIM_OCInitStructure);/*初始化通道1*/

TIM_OCInitStructure.TIM_Pulse = 5000;/*设置比较寄存器*/
TIM_OC2Init(TIM1,&TIM_OCInitStructure);/*初始化通道2*/

TIM_OCInitStructure.TIM_Pulse = 5000;/*设置比较寄存器*/
TIM_OC3Init(TIM1,&TIM_OCInitStructure);/*初始化通道3*/

TIM_OCInitStructure.TIM_Pulse = 5000;/*设置比较寄存器*/
TIM_OC4Init(TIM1,&TIM_OCInitStructure);/*初始化通道4*/


TIM_ARRPreloadConfig(TIM1, ENABLE);   //默认打开，可以不用设置
TIM_Cmd(TIM1,ENABLE);/*计数使能*/
TIM_CtrlPWMOutputs(TIM1,ENABLE);/*输出使能*/

}


void update_SingleMotorState(u8 Motor,u8 State)   //更新单个电机的状态：开启-停止-自由
{
  switch(Motor)
	{
	  case 0:
			if(State==Run)               //state为start=1，启动
			{GPIO_ResetBits(GPIOD,GPIO_Pin_14);
			      TIM1->CCR4=PWM_Max/2;}
			else if(State==Stop)                        //为0，停止
			{	GPIO_SetBits(GPIOD,GPIO_Pin_14);
			      TIM1->CCR4=PWM_Min;}    
       else if(State==Release)                        //为2，电机自由
			{	GPIO_SetBits(GPIOD,GPIO_Pin_14);
			      TIM1->CCR4=PWM_Max-1;}  			
		break;
		
		 case 1:
			 if(State==Run)               //state为start=1，启动
			{GPIO_ResetBits(GPIOD,GPIO_Pin_10);
			      TIM1->CCR3=PWM_Max/2;}
			else if(State==Stop)                   //为0，停止
			{	GPIO_SetBits(GPIOD,GPIO_Pin_10);
			      TIM1->CCR3=PWM_Min;}  
      	else if(State==Release)                  //为2，电机自由
			{	GPIO_SetBits(GPIOD,GPIO_Pin_10);
			      TIM1->CCR3=PWM_Max-1;}  		
		break;
		 
		  case 2:
				if(State==Run)                 //state为start=1，启动
			{GPIO_ResetBits(GPIOC,GPIO_Pin_8);
			      TIM1->CCR2=PWM_Max/2;}
			else if(State==Stop)              //为0，停止
			{	GPIO_SetBits(GPIOC,GPIO_Pin_8);
			      TIM1->CCR2=PWM_Min;}  
       else if(State==Release)                  //为2，电机自由		
			 {GPIO_SetBits(GPIOC,GPIO_Pin_8);
			      TIM1->CCR2=PWM_Max-1;}				 
		break;
			
			 case 3:
				 if(State==Run)              //state为start=1，启动
			{GPIO_ResetBits(GPIOD,GPIO_Pin_8);
			      TIM1->CCR1=PWM_Max/2;}
		else if(State==Stop)           //为0，停止
			{	GPIO_SetBits(GPIOD,GPIO_Pin_8);
			      TIM1->CCR1=PWM_Min;}  
     else if(State==Release)                  //为2，电机自由		
		  {GPIO_SetBits(GPIOD,GPIO_Pin_8);
			      TIM1->CCR1=PWM_Max-1;}			 
		break;
			 default:
				 break;
	}
}
void update_AllMotorState(void)   //更新所有电机的状态
{
  for(u8 i=0;i<4;i++)  update_SingleMotorState(i,AMT_Car.Motor[i].State);
}

void Stop_AllMotor(void)
{
  for(u8 i=0;i<4;i++) AMT_Car.Motor[i].State=Stop;
	update_AllMotorState();
}
void Run_AllMotor(void)
{
 for(u8 i=0;i<4;i++) AMT_Car.Motor[i].State=Run;
	update_AllMotorState();
	reset_MotorController(); //电机重新启动时，PID中间参数需要清零
}
void Release_AllMotor(void)
{
 for(u8 i=0;i<4;i++) AMT_Car.Motor[i].State=Release;
	update_AllMotorState();
}
void update_PWM(void)
{
 // for(u8 i=0;i<4;i++) Motor[i].Motor_PWM=constrain(Motor[i].Motor_PWM,PWM_Min,PWM_Max-1);   //限幅
	//更新PWM
	TIM1->CCR4=AMT_Car.Motor[0].Motor_PWM;
	TIM1->CCR3=AMT_Car.Motor[1].Motor_PWM;
	TIM1->CCR2=AMT_Car.Motor[2].Motor_PWM;
	TIM1->CCR1=AMT_Car.Motor[3].Motor_PWM;
}
