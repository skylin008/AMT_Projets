#ifndef __MPU6050_H
#define __MPU6050_H
#include "stm32f1xx_hal.h"

#define Windows_Filter_NUM        8     //滑动窗口滤波窗口数


#define	MPU6050_devAddr  	        0xD0	//MPU6050 I2C设备地址
#define MPU6050_WHO_AM_I_ID         0x68    //MPU6050自身ID
/*******************MPU6050寄存器列表***************/
#define MPU6050_REG_WHO_AM_I        0x75  //       
#define	MPU6050_REG_PWR_MGMT_1		0x6B  //电源管理1，时钟源，装置复位，温度传感器使能0，典型值：0x00(正常启用)
#define	MPU6050_REG_SMPLRT_DIV		0x19  //采样分频
#define	MPU6050_REG_MPU_CONFIG	    0x1A  //不使能FSYNC(外部预采样同步信号)；DLPF_CFG[2~0],设置任意轴是否通过DLPF，典型值：0x06(5Hz)低通滤波器带宽5Hz，
														//对加速度和陀螺仪都有效，输出频率为1kHz，决定SMPLRT_DIV的频率基准
#define	MPU6050_REG_GYRO_CONFIG		0x1B					//陀螺仪配置寄存器；陀螺仪自检及量程，可以单独使能任意轴自检[7~5]；FS_SEL[4~3],00+-250，01+—500，10+-1000，11+-2000
														//典型值：0x18(不自检，最大量程2000deg/s)
#define	MPU6050_REG_ACCEL_CONFIG	0x1C					//加速度计配置寄存器，自检及量程，可以单独使能任意轴自检；DHPF数字高通滤波频率，典型值：0x0c(不自检，16G，5Hz)
#define MPU6050_REG_INT_PIN_CFG     0x37              //INT引脚中断电平，0高有效，1为低电平有效；上拉与推挽，边缘触发（50us的脉冲）或者电平触发（直到中断清除）；只有读该寄存器是否清中断。。。
#define MPU6050_REG_INT_ENABLE      0x38              //运动检测使能，FIFO溢出是否产生中断，DATA_ready中断使能。。。(更新所有数据寄存器后产生)
#define MPU6050_REG_INT_STATUS      0x3A              //对应上面的中断源，只读
#define	ACCEL_XOUT_H	0x3B					//加速度数据寄存器六个字节
#define	ACCEL_XOUT_L	0x3C					//以采样速率写入数据寄存器
#define	ACCEL_YOUT_H	0x3D					//16位二进制补码形式存储
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41				   //温度传感器数据寄存器，以采样速率写入数据，和以上数据寄存器，外部传感器数据寄存器
#define	TEMP_OUT_L		0x42				   //组成两种寄存器：内部寄存器，用户可读寄存器，总线接口空闲时，以上数据寄存器数据更新到可读寄存器，
														//在采用突发读取寄存器的数值时，实际上读的是可读寄存器的值，以保证读的数据是同一采用时刻的高低字节的
														//数据，以免高低数据字节不对应

#define	GYRO_XOUT_H		0x43					//陀螺仪数据寄存器的六个字节；
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

typedef struct
{
  int16_t X;
  int16_t Y;
  int16_t Z;
}Senor_t;
typedef struct
{
  float Roll;
  float Pitch;
  float Yaw;
}Axis_t;
typedef struct
{
  Senor_t ACC;             //加速度三轴原始数据
  Senor_t ACC_OffSet;
  Senor_t Gyro;            //陀螺仪三轴原始数据
  Senor_t Gyro_OffSet;
  Axis_t Axis_Angle;       //数据融合角度
  Axis_t Axis_Rate;        //数据融合角速度
  int16_t Temp;
}MPU6050_t;



extern MPU6050_t MPU6050; 


void init_MPU6050(void);
void get_MPU6050_RawData(MPU6050_t *MPU6050);
void ACC_Windows_Filter(MPU6050_t *MPU6050);
uint8_t calibrateSensor(MPU6050_t *MPU6050,uint8_t BUFNUM,uint8_t Sensor,int16_t maxDifferent);


#endif
