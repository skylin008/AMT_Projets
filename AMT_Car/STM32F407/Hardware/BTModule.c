#include "BTModule.h"
#include <stdio.h>

__IO uint8_t nRx2Counter=0; //接收字节数
__IO uint8_t USART_Rx2Buff[FRAME_BYTE_LENGTH]; //接收缓冲区
__IO uint8_t USART_FrameFlag = 0; //接收完整数据帧标志，1完整，0不完整
__IO uint8_t i;

void USART_SendByte(USART_TypeDef* USARTx,u8 data)
{
  USART_SendData(USARTx,data);
  while(USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);	
}
void USART_Send2Byte(USART_TypeDef* USARTx,u16 data)
{
  USART_SendByte(USARTx,(u8)data);   //先发送低八位
	USART_SendByte(USARTx,(u8)(data>>8));   //发送高八位
}

void init_USART2(void)
{
	 //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
 
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

}

/******************************************************
		格式化串口输出函数
        "\r"	回车符	   USART_OUT(USART1, "abcdefg\r")   
		"\n"	换行符	   USART_OUT(USART1, "abcdefg\r\n")
		"%s"	字符串	   USART_OUT(USART1, "字符串是：%s","abcdefg")
		"%d"	十进制	   USART_OUT(USART1, "a=%d",10)
**********************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){ 
	const char *s;
    int d;
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0){				                          //判断是否到达字符串结束符
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //回车符
					USART_SendData(USARTx, 0x0d);	   
					Data++;
					break;
				case 'n':							          //换行符
					USART_SendData(USARTx, 0x0a);	
					Data++;
					break;
				
				default:
					Data++;
				    break;
			}
	
			 
		}
		else if(*Data=='%'){									  //
			switch (*++Data){				
				case 's':										  //字符串
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
            	case 'd':										  //十进制
                	d = va_arg(ap, int);
                	itoa(d, buf, 10);
                	for (s = buf; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
				default:
					Data++;
				    break;
			}		 
		}
		else USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}
/******************************************************
		整形数据转字符串函数
        char *itoa(int value, char *string, int radix)
		radix=10 标示是10进制	非十进制，转换结果为0;  

	    例：d=-379;
		执行	itoa(d, buf, 10); 后
		
		buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */

//重定向printf到串口
      

//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}

void USART_GetChar(uint8_t nChar) //串口接收到一个字节
{

	if(USART_FrameFlag == 1) return;   //如果上次的数据帧还没处理过，则返回
  
	if(nRx2Counter==0 && nChar == FRAME_START)
	{
		USART_Rx2Buff[nRx2Counter++]=nChar;  //保存到缓冲区
	}
		else if(nRx2Counter>0) //接收到帧头以后才继续保存
	{
	  USART_Rx2Buff[nRx2Counter++]=nChar;  //保存到缓冲区
		if(nRx2Counter>=FRAME_BYTE_LENGTH)  //接收到一帧数据
		{
			nRx2Counter = 0;
			if(USART_Rx2Buff[FRAME_BYTE_LENGTH-1] == FRAME_END) //如果最后一个字节是帧尾，则数据帧完整
			{
				USART_FrameFlag=1;
			}
		}
	
	
	
	}
}

void USART_Process(void) //处理数据帧
{
/*
	//uint16_t nVal;
	//float fKp,fKi,fKd;
	uint8_t YAW_KP=0,YAW_KI=0,YAW_KD=0;
	if(USART_FrameFlag == 1)
	{
		//printf("jinru\r\n");
		//printf("%d",USART_Rx2Buff[4]);
		if(USART_Rx2Buff[3]==0xF1)
		  YAW_KP=USART_Rx2Buff[4];
		if(USART_Rx2Buff[3]==0xF2)
		  YAW_KI=USART_Rx2Buff[4];
	 if(USART_Rx2Buff[3]==0xF3)
		  YAW_KD=USART_Rx2Buff[4];
		
		
		
		
	//	YAW_SetPID(YAW_KP,YAW_KI,YAW_KD);
		USART_Fr56Y6YameFlag = 0;
		
		
		
		
		
		}
		
		*/
		//将数据原封不动发送回去
//		for(i=0;i<FRAME_BYTE_LENGTH;i++)
//		{
//			USART_SendData(USART2,USART_Rx2Buff[i]);
//			while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
//		}
		/*
		if(USART_Rx2Buff[1] == 0x11) //如果命令字节等于0x11，则是设置PID参数指令，这些协议可以自己定义
		{
			//Kp值
			nVal = (USART_Rx2Buff[2]<<8) + USART_Rx2Buff[3];
			fKp = (float)nVal/100.0;   //传输时放大100倍，譬如Kp=2.3，传输时为230整数
			//Ki值
			nVal = (USART_Rx2Buff[4]<<8) + USART_Rx2Buff[5];
			fKi = (float)nVal/100.0;
			//Kp值
			nVal = (USART_Rx2Buff[6]<<8) + USART_Rx2Buff[7];
			fKp = (float)nVal/100.0;
			MotorController_SetPIDParam(fKp,fKi,fKd);
		}
		else if(USART_Rx2Buff[1] == 0x12) //如果命令字节等于0x12，则是设置加速度参数指令，这些协议可以自己定义
		{
			//2、3字节是加速度值
			nVal = (USART_Rx2Buff[2]<<8) + USART_Rx2Buff[3];
			MotorController_SetAcceleration(nVal);
		}
		//处理完毕，将标志清0
		USART_FrameFlag = 0;   */
	
}


/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)      //串口1中断服务
{
	uint8_t clear=clear;
	uint8_t ch;
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)	   //判断读寄存器是否为空
  {	
		//USART_ClearITPendingBit(USART2,USART_IT_RXNE);      //清除中断标志
    /* Read one byte from the receive data register */
		//getchar = USART_ReceiveData(USART2);   //将读寄存器的数据缓存到接收缓冲区里	
		ch=USART2->DR;
		USART_GetChar(ch);
  }

 if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)                   //这段是为了避免STM32 USART第一个字节发送不出去的BUG 
  { 	
     USART_ITConfig(USART2, USART_IT_TXE, DISABLE);					     //禁止发送缓冲器空中断
	}	
  
}
