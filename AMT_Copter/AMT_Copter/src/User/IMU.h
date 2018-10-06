#ifndef __IMU_H
#define __IMU_H
#include "stm32f1xx_hal.h"
#include "MPU6050.h"





void update_IMU(float dt);
void Kalman_Filter_X(float Accel,float Gyro,MPU6050_t *MPU6050,float dt);//卡尔曼函数	
void Kalman_Filter_Y(float Accel,float Gyro,MPU6050_t *MPU6050,float dt);//卡尔曼函数	


#endif
