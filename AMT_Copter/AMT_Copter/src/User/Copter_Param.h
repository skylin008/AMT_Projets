#ifndef __COPTER_PARAM_H
#define __COPTER_PARAM_H

//
#define USE_EZ_GUI        //使用手机APP EZ_GUI操控
//#define  USE_TELE_Control    //使用遥控器
//#define USE_MPU6050_Temp  //使用MPU6050的温度

#define maxAngle_Stable         30           //自稳最大角度     
/*************************PID参数**************************/
//外环
#define headKp                  1.0f         //锁头模式下外环
#define stickScalingYaw         3.5f         //Yaw角外环 
#define AngleKp                 5.5f         //Roll和Pitch共用一组外环P
//内环
#define pidYaw_Kp         4.0f               //Yaw内环Kp
#define pidYaw_Ki         0.0f               //Yaw内环Ki
#define pidYaw_Kd         0.0f               //Yaw内环Kd
#define pidRollPitch_Kp         1.8f         //Roll，Pitch内环Kp
#define pidRollPitch_Ki         0.0f         //Roll，Pitch内环Ki
#define pidRollPitch_Kd         0.02f         //Roll，Pitch内环Kd
#define iTerm_LimitYaw          5.0f         //Yaw内环积分阈值
#define iTerm_LimitRollPitch    50.0f         //Roll,Pitch内环积分阈值








#endif
