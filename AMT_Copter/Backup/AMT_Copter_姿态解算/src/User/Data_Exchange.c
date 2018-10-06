#include "Data_Exchange.h"
#include "usart.h"

uint8_t RX_Buffer,RX_Frame=0,RX_Flag=0,RX_DATA[22]; //串口数据接收缓存区,已接收字节,接收完成标志位，接收完成数据存储
Rx_Channel_t Rx_Channel;


/**
  * @brief  串口初始化函数
  * @param  None
  * @retval None
  */
void init_RX(void)
{
  HAL_UART_Receive_IT(&huart3,&RX_Buffer,1);
}

/**
  * @brief  接收通道数据合成
  * @param  None
  * @retval None
  */
void DATA_Compose(void)
{
  Rx_Channel.Rx_Roll=(uint16_t)(RX_DATA[6]<<8|RX_DATA[5]);
  Rx_Channel.Rx_Pitch=(uint16_t)(RX_DATA[8]<<8|RX_DATA[7]);
  Rx_Channel.Rx_Yaw=(uint16_t)(RX_DATA[10]<<8|RX_DATA[9]);
  Rx_Channel.Rx_Thr=(uint16_t)(RX_DATA[12]<<8|RX_DATA[11]);
  Rx_Channel.Rx_AUX1=(uint16_t)(RX_DATA[14]<<8|RX_DATA[13]);
  Rx_Channel.Rx_AUX2=(uint16_t)(RX_DATA[16]<<8|RX_DATA[15]);
  Rx_Channel.Rx_AUX3=(uint16_t)(RX_DATA[18]<<8|RX_DATA[17]);
  Rx_Channel.Rx_AUX4=(uint16_t)(RX_DATA[20]<<8|RX_DATA[19]);

#if 0
   //printf("%d %d %d %d\n",Rx_Channel.Rx_Roll,Rx_Channel.Rx_Pitch,Rx_Channel.Rx_Thr,Rx_Channel.Rx_Yaw);
   printf("%d %d %d %d\n",Rx_Channel.Rx_AUX1,Rx_Channel.Rx_AUX2,Rx_Channel.Rx_AUX3,Rx_Channel.Rx_AUX4);
#endif
}
/**
  * @brief  返回数据接收标志位
  * @param  None
  * @retval RX_Flag
  */
uint8_t get_Rx_Flag(void)
{
  return RX_Flag;
}
void RESET_Rx_Flag(void)
{
  RX_Flag=0;
}
//检查遥控器指令是否接收完成
  // if(get_Rx_Flag())
  // {
  //   RESET_Rx_Flag();  //清除标志位
  //   DATA_Compose();   //指令解算
  // }
/**
  * @brief  串口中断回调函数
  * @param  *huart
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART3)       //判断中断源
  {
    //HAL_UART_Transmit_IT(&huart2,&RX_Temp,1);
    if(RX_Frame==0&&RX_Flag==0)     //上一帧已处理完成，寻找头帧
    {
       if(RX_Buffer==FIRST_FRAME)    //头帧24
       {
         RX_DATA[RX_Frame]=RX_Buffer;
         RX_Frame++;
       }
    } 
    else 
    {
    if(RX_Frame==1)   //4D
       {
           if(RX_Buffer==0x4d)   //第二帧
           {
              RX_DATA[RX_Frame]=RX_Buffer;
              RX_Frame++;
           }
           else RX_Frame=0;         
       }
    else if(RX_Frame==2)   //3C
       {
           if(RX_Buffer==0x3c)   //第三帧
           {
             RX_DATA[RX_Frame]=RX_Buffer;
             RX_Frame++;
           }
           else RX_Frame=0;         
       }
    else if(RX_Frame==3)   //
       {
           if(RX_Buffer==0x10)   //第四帧
           {
             RX_DATA[RX_Frame]=RX_Buffer;
             RX_Frame++;
           }
           else RX_Frame=0;         
       }
    else if(RX_Frame==4)   //
       {
           if(RX_Buffer==0xc8)   //第五帧
           {
             RX_DATA[RX_Frame]=RX_Buffer;
             RX_Frame++;
           }
           else RX_Frame=0;         
       }
    else if(RX_Frame>=5)   //
       {        
          RX_DATA[RX_Frame]=RX_Buffer;
          RX_Frame++;
        if(RX_Frame>=21)  //接收完成
        {
          RX_Frame=0;
          RX_Flag=SET;   //接收完成标志位置1
          DATA_Compose();   //指令解算
        }
       }
    }
    HAL_UART_Receive_IT(&huart3,&RX_Buffer,1);  //继续打开接收中断
  }
}
