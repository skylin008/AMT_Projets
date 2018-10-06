#ifndef __DATA_EXCHANGE_H
#define __DATA_EXCHANGE_H
#include "stm32f1xx_hal.h"

#define FIRST_FRAME 0x24
#define Mincheck    1020
#define Maxcheck   1980


typedef struct
{
  uint16_t Rx_Roll;
  uint16_t Rx_Pitch;
  uint16_t Rx_Thr;
  uint16_t Rx_Yaw;
  uint16_t Rx_AUX1;         //定高开关
  uint16_t Rx_AUX2;         //
  uint16_t Rx_AUX3;
  uint16_t Rx_AUX4;
}Rx_Channel_t;


extern Rx_Channel_t Rx_Channel;


void init_RX(void);
uint8_t get_Rx_Flag(void);
void RESET_Rx_Flag(void);
void DATA_Compose(void);    //接收通道数据合成


#endif



