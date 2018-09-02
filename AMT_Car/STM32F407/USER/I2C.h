#ifndef __I2C_H
#define __I2C_H
#include "stm32f10x.h"
#include "delay.h"

//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))  

//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入 



//I2C_IO方向设置
#define SDA_IN()  {GPIOC->CRH&=0XFF0FFFFF;GPIOC->CRH|=8<<20;}
#define SDA_OUT() {GPIOC->CRH&=0XFF0FFFFF;GPIOC->CRH|=3<<20;}

//IO操作函数	 
#define I2C_SCL    PCout(0) //SCL
#define I2C_SDA    PCout(13) //SDA	 
#define READ_SDA   PCin(13)  //输入SDA 
#define Delay() I2C_Delay(65)

void I2C_Delay(uint16_t i);
void I2C_Config(void);
int	I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NAck(void);
int I2C_WaitAck(void);
////////////////////////////////////////////
void I2C_SendByte(u8 txd);
u8 I2C_ReadByte(unsigned char ack);
/////////////////////////////////////////////
u8 I2C_WriteBytes(u8 dev, u8 reg, u8 length, u8* data);
unsigned char I2C_WriteOneByte(unsigned char dev, unsigned char reg, unsigned char data);
///////////////////////////////////////////////
void I2C_readBytes(u8 dev, u8 reg, u8 length, uint8_t *data);
uint8_t I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);



int GetData(char REG_Address);
#endif
