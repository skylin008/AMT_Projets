#include "Encoder.h"
#include "BTModule.h"
#include "CarController.h"


/*
  EA-A---PB4(TIM3_CH1);EA-B---PB5(TIM3_CH2)
  EB-A---PA15(TIM2_CH1);EB-B---PB3(TIM2_CH2)
	EC-A---PC6(TIM8_CH1);EC-B---PC7(TIM8_CH2)
	ED-A---PD12(TIM4_CH1);ED-B---PD13(TIM4_CH2)
*/
void init_Encoder(void)
{
  GPIO_InitTypeDef         GPIO_InitStructure; 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 // TIM_ICInitTypeDef        TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  //NVIC�ж������ṹ�� 
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4,ENABLE);//����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);/*ʹ�ܶ�ʱ��8��ʱ��*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);//����GPIOBʱ��
  
	/*********GPIO����*********/
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);//PB4���Ÿ���
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);//PB5���ŷ���
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2);//
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2);//
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);//
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);//
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);//
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);//	
	
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;  
  GPIO_Init(GPIOB,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13;
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
 
    TIM_TimeBaseStructure.TIM_Period = ENC_ARR-1; //������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
		
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
		TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
 
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising); //������ģʽ�£�CNT�Զ�����
		TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
		TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
		TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
		
//    TIM_ICStructInit(&TIM_ICInitStructure);
//    TIM_ICInitStructure.TIM_ICFilter = 0;  //�����˲���
//    TIM_ICInit(TIM3, &TIM_ICInitStructure);
		
			/* Enable the Update Interrupt */
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//TIM3
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	
		NVIC_Init(&NVIC_InitStructure);
						
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);  //������б�־λ
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //�����жϸ���
    TIM3->CNT = 0;
    TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3			
//TIM2
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	
		NVIC_Init(&NVIC_InitStructure);
						
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);  //������б�־λ
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //�����жϸ���
    TIM2->CNT = 0;
    TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM3		
//TIM8		
	  NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;	
		NVIC_Init(&NVIC_InitStructure);
						
    TIM_ClearFlag(TIM8, TIM_FLAG_Update);  //������б�־λ
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE); //�����жϸ���
    TIM8->CNT = 0;
    TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM3			
//TIM4
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	
		NVIC_Init(&NVIC_InitStructure);
						
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);  //������б�־λ
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //�����жϸ���
    TIM4->CNT = 0;
    TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM3			
}


void update_MotorSpeed(void)  //�����ٶ�
{
	//���±�����
	AMT_Car.Motor[0].Encoder_CNT_Cur=AMT_Car.Motor[0].Encoder_Overflow_CNT*ENC_ARR+TIM3->CNT;
	AMT_Car.Motor[1].Encoder_CNT_Cur=AMT_Car.Motor[1].Encoder_Overflow_CNT*ENC_ARR+TIM2->CNT;
	AMT_Car.Motor[2].Encoder_CNT_Cur=AMT_Car.Motor[2].Encoder_Overflow_CNT*ENC_ARR+TIM8->CNT;
  AMT_Car.Motor[3].Encoder_CNT_Cur=AMT_Car.Motor[3].Encoder_Overflow_CNT*ENC_ARR+TIM4->CNT;
	//�����ٶ�
	for(u8 i=0;i<4;i++)  //�ĸ�����
	{
		//Motor[i].SpeedLast=Motor[i].SpeedCur;
		AMT_Car.Motor[i].SpeedCur=(float)3.14159*WheelDiameter*(AMT_Car.Motor[i].Encoder_CNT_Cur-AMT_Car.Motor[i].Encoder_CNT_Last)/(EncoderResolution*4*DT);
		AMT_Car.Motor[i].Encoder_CNT_Last=AMT_Car.Motor[i].Encoder_CNT_Cur;		
	}
	
}

void TIM3_IRQHandler(void)       //Encoder_A   //����ж�
{
	 //�Ƿ��и����ж�
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)
	{
		 //����жϱ�־
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update); 
		//�����ж�
		if(TIM3->CNT>ENC_ARR/2) AMT_Car.Motor[0].Encoder_Overflow_CNT--;    //��0->ENC_ARR,����������ʼ�1
		else AMT_Car.Motor[0].Encoder_Overflow_CNT++;    //��ENC_ARR->0,����������ʼ�1
	}	
}
void TIM2_IRQHandler(void)       //Encoder_B        
{
	 //�Ƿ��и����ж�
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
	{
		 //����жϱ�־
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update); 
		//�����ж�
		if(TIM2->CNT>ENC_ARR/2) AMT_Car.Motor[1].Encoder_Overflow_CNT--;    //��0->ENC_ARR,����������ʼ�1
		else AMT_Car.Motor[1].Encoder_Overflow_CNT++;    //��ENC_ARR->0,����������ʼ�1
	}	
}
void TIM8_CC_IRQHandler(void)       //Encoder_C
{
	 //�Ƿ��и����ж�
	if(TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)
	{
		 //����жϱ�־
		TIM_ClearITPendingBit(TIM8,TIM_IT_Update); 
		//�����ж�
		if(TIM8->CNT>ENC_ARR/2) AMT_Car.Motor[2].Encoder_Overflow_CNT--;    //��0->ENC_ARR,����������ʼ�1
		else AMT_Car.Motor[2].Encoder_Overflow_CNT++;    //��ENC_ARR->0,����������ʼ�1
	}	
}
void TIM4_IRQHandler(void)       //Encoder_D
{
	 //�Ƿ��и����ж�
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
	{
		 //����жϱ�־
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
		//�����ж�
		if(TIM4->CNT>ENC_ARR/2) AMT_Car.Motor[3].Encoder_Overflow_CNT--;    //��0->ENC_ARR,����������ʼ�1
		else AMT_Car.Motor[3].Encoder_Overflow_CNT++;    //��ENC_ARR->0,����������ʼ�1
	}	
}
