#ifndef __Encoder_H
#define __Encoder_H
#include "stm32f4xx.h" 
#include "MotorController.h"

#define ENC_ARR 65535
#define WheelDiameter 58   //轮子直径（mm）
#define EncoderResolution 330  //编码器分辨率



void init_Encoder(void);
void update_MotorSpeed(void);       //计算速度

#endif
