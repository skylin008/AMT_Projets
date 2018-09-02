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
	
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);/*ʹ��GPIOA��ʱ��*/
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);/*ʹ�ܶ�ʱ��1��ʱ��*/
/****************GPIO����***************/
	//IN1_GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                   //����
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_14 ;	
  GPIO_Init(GPIOD, &GPIO_InitStructure);                         //��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
  GPIO_Init(GPIOC, &GPIO_InitStructure);       
	
	//IN2_GPIO����
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 ;	
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/*����*/
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/*�������*/
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;/**/
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/**/

GPIO_Init(GPIOA,&GPIO_InitStructure);/*��ʼ��IO*/

GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);    
GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);    
GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);    
GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1);    

TIM_TimeBaseInitStrecture.TIM_Prescaler = 0;    //(Ƶ��)f=Tclk/((arr+1)*(psc+1))  //�˴�16.8KHZ
TIM_TimeBaseInitStrecture.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
TIM_TimeBaseInitStrecture.TIM_Period = 9999;/*�����Զ���װ*/
TIM_TimeBaseInitStrecture.TIM_ClockDivision = TIM_CKD_DIV1;/**/
TIM_TimeBaseInitStrecture.TIM_RepetitionCounter = 0;/**/
TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStrecture);/*��ʼ��*/

/************************PWMģʽ����************************/
TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*����PWMģʽ*/
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//���ʹ��
TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;    //ȱ��һ��TIM1-CH1~3���������
TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;  //TIM1�������
TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Reset;


TIM_OCInitStructure.TIM_Pulse = 5000;/*���ñȽϼĴ���*/
TIM_OC1Init(TIM1,&TIM_OCInitStructure);/*��ʼ��ͨ��1*/

TIM_OCInitStructure.TIM_Pulse = 5000;/*���ñȽϼĴ���*/
TIM_OC2Init(TIM1,&TIM_OCInitStructure);/*��ʼ��ͨ��2*/

TIM_OCInitStructure.TIM_Pulse = 5000;/*���ñȽϼĴ���*/
TIM_OC3Init(TIM1,&TIM_OCInitStructure);/*��ʼ��ͨ��3*/

TIM_OCInitStructure.TIM_Pulse = 5000;/*���ñȽϼĴ���*/
TIM_OC4Init(TIM1,&TIM_OCInitStructure);/*��ʼ��ͨ��4*/


TIM_ARRPreloadConfig(TIM1, ENABLE);   //Ĭ�ϴ򿪣����Բ�������
TIM_Cmd(TIM1,ENABLE);/*����ʹ��*/
TIM_CtrlPWMOutputs(TIM1,ENABLE);/*���ʹ��*/

}


void update_SingleMotorState(u8 Motor,u8 State)   //���µ��������״̬������-ֹͣ-����
{
  switch(Motor)
	{
	  case 0:
			if(State==Run)               //stateΪstart=1������
			{GPIO_ResetBits(GPIOD,GPIO_Pin_14);
			      TIM1->CCR4=PWM_Max/2;}
			else if(State==Stop)                        //Ϊ0��ֹͣ
			{	GPIO_SetBits(GPIOD,GPIO_Pin_14);
			      TIM1->CCR4=PWM_Min;}    
       else if(State==Release)                        //Ϊ2���������
			{	GPIO_SetBits(GPIOD,GPIO_Pin_14);
			      TIM1->CCR4=PWM_Max-1;}  			
		break;
		
		 case 1:
			 if(State==Run)               //stateΪstart=1������
			{GPIO_ResetBits(GPIOD,GPIO_Pin_10);
			      TIM1->CCR3=PWM_Max/2;}
			else if(State==Stop)                   //Ϊ0��ֹͣ
			{	GPIO_SetBits(GPIOD,GPIO_Pin_10);
			      TIM1->CCR3=PWM_Min;}  
      	else if(State==Release)                  //Ϊ2���������
			{	GPIO_SetBits(GPIOD,GPIO_Pin_10);
			      TIM1->CCR3=PWM_Max-1;}  		
		break;
		 
		  case 2:
				if(State==Run)                 //stateΪstart=1������
			{GPIO_ResetBits(GPIOC,GPIO_Pin_8);
			      TIM1->CCR2=PWM_Max/2;}
			else if(State==Stop)              //Ϊ0��ֹͣ
			{	GPIO_SetBits(GPIOC,GPIO_Pin_8);
			      TIM1->CCR2=PWM_Min;}  
       else if(State==Release)                  //Ϊ2���������		
			 {GPIO_SetBits(GPIOC,GPIO_Pin_8);
			      TIM1->CCR2=PWM_Max-1;}				 
		break;
			
			 case 3:
				 if(State==Run)              //stateΪstart=1������
			{GPIO_ResetBits(GPIOD,GPIO_Pin_8);
			      TIM1->CCR1=PWM_Max/2;}
		else if(State==Stop)           //Ϊ0��ֹͣ
			{	GPIO_SetBits(GPIOD,GPIO_Pin_8);
			      TIM1->CCR1=PWM_Min;}  
     else if(State==Release)                  //Ϊ2���������		
		  {GPIO_SetBits(GPIOD,GPIO_Pin_8);
			      TIM1->CCR1=PWM_Max-1;}			 
		break;
			 default:
				 break;
	}
}
void update_AllMotorState(void)   //�������е����״̬
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
	reset_MotorController(); //�����������ʱ��PID�м������Ҫ����
}
void Release_AllMotor(void)
{
 for(u8 i=0;i<4;i++) AMT_Car.Motor[i].State=Release;
	update_AllMotorState();
}
void update_PWM(void)
{
 // for(u8 i=0;i<4;i++) Motor[i].Motor_PWM=constrain(Motor[i].Motor_PWM,PWM_Min,PWM_Max-1);   //�޷�
	//����PWM
	TIM1->CCR4=AMT_Car.Motor[0].Motor_PWM;
	TIM1->CCR3=AMT_Car.Motor[1].Motor_PWM;
	TIM1->CCR2=AMT_Car.Motor[2].Motor_PWM;
	TIM1->CCR1=AMT_Car.Motor[3].Motor_PWM;
}
