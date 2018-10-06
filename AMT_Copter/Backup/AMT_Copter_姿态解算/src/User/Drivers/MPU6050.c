#include "MPU6050.h"
#include "i2c.h"
#include "Copter_Param.h"
#include "usart.h"

#define ACC_X_OffSet  6
#define ACC_Y_OffSet  212
#define ACC_Z_OffSet  68
#define Gyro_X_OffSet -124
#define Gyro_Y_OffSet -21
#define Gyro_Z_OffSet 22

MPU6050_t MPU6050;
int16_t Windows_Filter_Buffer[3][Windows_Filter_NUM+1];              //滑动窗口滤波数组缓存3行

/**
  * @brief  寻找MPU6050设备
  * @param  None
  * @retval None
  */
void get_Device_MPU6050(void)
{
    uint8_t buffer;
  if(HAL_I2C_Mem_Read(&hi2c1,MPU6050_devAddr,MPU6050_REG_WHO_AM_I,I2C_MEMADD_SIZE_8BIT,&buffer,1,0xff)==HAL_OK)
  {
      if(buffer==MPU6050_WHO_AM_I_ID)
         printf("**********************Found MPU6050********************\n");
      else
      {
          printf("****************Not Found MPU6050*****************\n");
          while(1) {}
      } 
  }
  else   //I2C出错
  {
     printf("*****************I2C Bus ERROR*****************\n");
     while(1) {}
  }
}
//MPU6050写数据
void MPU6050_REG_Write(uint8_t REG,uint8_t Data)
{
  HAL_I2C_Mem_Write(&hi2c1,MPU6050_devAddr,REG,I2C_MEMADD_SIZE_8BIT,&Data,1,0xff);
}
void MPU6050_REG_Read(uint8_t REG,uint8_t* pData,uint8_t Num)
{
  HAL_I2C_Mem_Read(&hi2c1,MPU6050_devAddr,REG,I2C_MEMADD_SIZE_8BIT,pData,Num,0xff);
}
/**
  * @brief  初始化MPU6050设备
  * @param  None
  * @retval None
  */
void init_MPU6050(void)
{
  get_Device_MPU6050();     //查找MPU6050设备
  
  //开始初始化MPU6050
  MPU6050_REG_Write(MPU6050_REG_PWR_MGMT_1,(1<<7));    //初始化MPU6050，写0x80
  HAL_Delay(100);
  uint8_t buffer=0x80;
  while(buffer&(1<<7)){MPU6050_REG_Read(MPU6050_REG_PWR_MGMT_1,&buffer,1);}  //等待电源管理位清除
  HAL_Delay(50);
  /***********/
  MPU6050_REG_Write(MPU6050_REG_SMPLRT_DIV,0);        //陀螺仪采样频率:1kHZ,原始数据更新频率为1kHZ数据更新
   /***********/
  MPU6050_REG_Write(MPU6050_REG_PWR_MGMT_1,0x03);   //  
   /***********/
  MPU6050_REG_Write(MPU6050_REG_MPU_CONFIG,0x03);    //Disable FSYNC and set 41 Hz Gyro filtering, 1 KHz sampling
   /***********/
  MPU6050_REG_Write(MPU6050_REG_GYRO_CONFIG,0x10);  //陀螺仪配置：满量程1000度/S
   /***********/
  MPU6050_REG_Write(MPU6050_REG_ACCEL_CONFIG,0x08); //加速度配置：满量程4g
   /***********/
   MPU6050_REG_Write(MPU6050_REG_INT_PIN_CFG,0x02);     
   //MPU6050_REG_Write(MPU6050_REG_INT_ENABLE,0x01);     
   
  HAL_Delay(100);
  while(!calibrateSensor(&MPU6050,25,1,10))
   {
   }
   
 // HAL_Delay(100);           //等待传感器稳定
  printf("******************MPU6050 Init Success*****************\n");
}
/**
  * @brief  获取MPU6050原始数据
  * @param  MPU6050_t
  * @retval None
  */
void get_MPU6050_RawData(MPU6050_t *MPU6050)
{
  uint8_t Buffer[14];

  MPU6050_REG_Read(ACCEL_XOUT_H,Buffer,14);
  MPU6050->ACC.X=(int16_t)(Buffer[0]<<8|Buffer[1])-ACC_X_OffSet;
  MPU6050->ACC.Y=(int16_t)(Buffer[2]<<8|Buffer[3])-ACC_Y_OffSet;
  MPU6050->ACC.Z=(int16_t)(Buffer[4]<<8|Buffer[5])-ACC_Z_OffSet;
  MPU6050->Gyro.X=(int16_t)(Buffer[8]<<8|Buffer[9])-MPU6050->Gyro_OffSet.X;
  MPU6050->Gyro.Y=(int16_t)(Buffer[10]<<8|Buffer[11])-MPU6050->Gyro_OffSet.Y;
  MPU6050->Gyro.Z=(int16_t)(Buffer[12]<<8|Buffer[13])-MPU6050->Gyro_OffSet.Z;
  
#ifdef USE_MPU6050_Temp
  int16_t Temp;
  Temp=(int16_t)(Buffer[6]<<8|Buffer[7]);
  MPU6050->Temp=((float)(1.0*Temp/340)+36.53)*10;
  #if 1
    printf("%d\n",MPU6050->Temp);
  #endif
#endif
#if 0
  int16_t buffer[3];
  buffer[0]=MPU6050->ACC.X;
  buffer[1]=MPU6050->ACC.Y;
  buffer[2]=MPU6050->ACC.Z;
  Wava_Send(buffer,3);
  //printf("%d %d %d\n",MPU6050->ACC.X,MPU6050->ACC.Y,MPU6050->ACC.Z);
#endif
#if 0
  // int16_t buffer[3];
  // buffer[0]=MPU6050->Gyro.X;
  // buffer[1]=MPU6050->Gyro.Y;
  // buffer[2]=MPU6050->Gyro.Z;
  // Wava_Send(buffer,3);
  printf("%d %d %d\n",MPU6050->Gyro.X,MPU6050->Gyro.Y,MPU6050->Gyro.Z);
#endif

}
/**
  * @brief  加速度计原始数据滑动窗口滤波
  * @param  MPU6050_t
  * @retval None
  */
void ACC_Windows_Filter(MPU6050_t *MPU6050)
{
  uint8_t i,j;
  //把数组数据前移一位
  for(i=1;i<Windows_Filter_NUM;i++)
  { 
    Windows_Filter_Buffer[0][i]=Windows_Filter_Buffer[0][i+1];
    Windows_Filter_Buffer[1][i]=Windows_Filter_Buffer[1][i+1];
    Windows_Filter_Buffer[2][i]=Windows_Filter_Buffer[2][i+1];
  }
  //最后一列存入新数据
  Windows_Filter_Buffer[0][Windows_Filter_NUM]=MPU6050->ACC.X;
  Windows_Filter_Buffer[1][Windows_Filter_NUM]=MPU6050->ACC.Y;
  Windows_Filter_Buffer[2][Windows_Filter_NUM]=MPU6050->ACC.Z;
  //滤波值存入第一列
  int32_t sum;
  for(i=0;i<3;i++)
  {
    sum=0;
    for(j=1;j<Windows_Filter_NUM+1;j++)
    {
      sum+=Windows_Filter_Buffer[i][j];
    }
    Windows_Filter_Buffer[i][0]=(float)(sum/Windows_Filter_NUM);
  }
  MPU6050->ACC.X=Windows_Filter_Buffer[0][0];
  MPU6050->ACC.Y=Windows_Filter_Buffer[1][0];
  MPU6050->ACC.Z=Windows_Filter_Buffer[2][0];

#if 0
  int16_t buffer[3];
  buffer[0]=MPU6050->ACC.X;
  buffer[1]=MPU6050->ACC.Y;
  buffer[2]=MPU6050->ACC.Z;
  Wava_Send(buffer,3);
  //printf("%d %d %d\n",MPU6050->ACC.X,MPU6050->ACC.Y,MPU6050->ACC.Z);
#endif
}

/**
  * @brief  传感器校零偏
  * @param  MPU6050_t
  * @retval None
  */
uint8_t calibrateSensor(MPU6050_t *MPU6050,uint8_t BUFNUM,uint8_t Sensor,int16_t maxDifferent)
{
  uint8_t Buffer[14],i,j;
  int16_t Calibrate_Buffer[3][BUFNUM],min,max;

  for(i=0;i<BUFNUM;i++)
  {
    MPU6050_REG_Read(ACCEL_XOUT_H,Buffer,14);
    
    if(Sensor==1)   //1:为陀螺仪
    {
      Calibrate_Buffer[0][i]=(int16_t)(Buffer[8]<<8|Buffer[9]);
      Calibrate_Buffer[1][i]=(int16_t)(Buffer[10]<<8|Buffer[11]);
      Calibrate_Buffer[2][i]=(int16_t)(Buffer[12]<<8|Buffer[13]);
    }
    else         //否则为加速度计
    {
      Calibrate_Buffer[0][i]=(int16_t)(Buffer[0]<<8|Buffer[1]);
      Calibrate_Buffer[1][i]=(int16_t)(Buffer[2]<<8|Buffer[3]);
      Calibrate_Buffer[2][i]=(int16_t)(Buffer[4]<<8|Buffer[5]);
    }
    HAL_Delay(10);
  }

  /**********************************************************/
  int32_t sum;
  //检查最大小差值是否在范围内
  for(i=0;i<3;i++)
  {
    min=max=Calibrate_Buffer[i][0];   
    for(j=0;j<BUFNUM;j++)
    {
       if(Calibrate_Buffer[i][j]>max) max=Calibrate_Buffer[i][j];
       else if(Calibrate_Buffer[i][j]<min) min=Calibrate_Buffer[i][j];
    }
    if((max-min)>maxDifferent) return 0;   //不在范围内继续更新，直到校准完毕
  }
  //求平均值
  for(i=0;i<3;i++)
  {
    sum=0;
    for(j=0;j<BUFNUM;j++)
    {
      sum+=Calibrate_Buffer[i][j];
    }
    if(Sensor==1)   //陀螺仪
    {
      if(i==0) MPU6050->Gyro_OffSet.X=(float)sum/BUFNUM;
      else if(i==1) MPU6050->Gyro_OffSet.Y=(float)sum/BUFNUM;
      else if(i==2) MPU6050->Gyro_OffSet.Z=(float)sum/BUFNUM;
    }
    else         //加速度计
    {

    }
   
  }
 return 1;
}

