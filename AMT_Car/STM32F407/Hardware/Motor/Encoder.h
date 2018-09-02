#ifndef __Encoder_H
#define __Encoder_H
#include "stm32f4xx.h" 
#include "MotorController.h"

#define ENC_ARR 65535
#define WheelDiameter 58   //����ֱ����mm��
#define EncoderResolution 330  //�������ֱ���



void init_Encoder(void);
void update_MotorSpeed(void);       //�����ٶ�

#endif
