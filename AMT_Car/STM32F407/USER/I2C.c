#include "I2C.h"


//初始化，PB8,PB9模拟I2C，推挽模式
/**************************实现函数********************************************
*函数原型:		void I2C_Config(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/

void I2C_Delay(uint16_t i)
{
	//uint32_t i;
	for(;i>0;i--);		/*1T单片机，12MHz晶振，延时时i=50能稳定地操作DRS1000
							如果换用更高速的单片机需要加大循环变量。*/
}
void I2C_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_13;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     
  GPIO_Init(GPIOC, &GPIO_InitStructure);					
}
//I2C起始信号
int I2C_Start(void)
{
	I2C_SCL=0;
	SDA_OUT();     //sda线输出
  I2C_SDA=1;
	Delay();
	
	if(!READ_SDA)return 0;	
	I2C_SCL=1;
	Delay();Delay();Delay();
	
	I2C_SDA=0;//START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	Delay();Delay();
	
	I2C_SCL=0;  //钳住I2C总线，准备发送或接收数据 
	Delay();
	
	return 1;
}
//I2C停止信号
void I2C_Stop(void)
{
	SDA_OUT();//sda线输出
	Delay();
  I2C_SCL=0;
	I2C_SDA=0;
	Delay();
	I2C_SCL=1;
	Delay();//Delay();Delay();
	
	I2C_SDA=1;//发送I2C总线结束信号
	Delay();Delay();
	I2C_SCL=0;
	//I2C_SCL_0();
}
//产生应答信号
//SCL为高时，SDA被拉低为应答，SDA被拉高为非应答
void I2C_Ack(void)
{
	I2C_SCL=0;
	SDA_OUT();
  I2C_SDA=0;
	Delay();
	
	I2C_SCL=1;
	Delay();
	
	I2C_SCL=0;
	Delay();

	I2C_SDA=1;   //CPU释放SDA
}
//产生非应答信号
void I2C_NAck(void)
{
	I2C_SCL=0;
	SDA_OUT();
  I2C_SDA=1;
	Delay();
	
	I2C_SCL=1;
	Delay();
	
	I2C_SCL=0;
	Delay();
}
//等待应答信号
int I2C_WaitAck(void)
{
  u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	Delay();
	I2C_SDA=1;           //CPU释放SDA总线
	Delay();

	I2C_SCL=1;           //CPU拉高SCL电平，发送一个时钟，此时会返回ACK应答
	Delay();
	
		while(READ_SDA)
	{
		
		ucErrTime++;
		if(ucErrTime>50)
		{
			I2C_Stop();
			return 0;
		}
	  Delay();
		
	}
	Delay();	
	I2C_SCL=0;   //时钟输出0 	   
Delay();	
	return 1;
}






///////////////////////////////////////////////////////////////////////
//I2C发送一个字节
void I2C_SendByte(u8 txd)
{
  u8 t;   
	SDA_OUT(); 	 

//	I2C_SCL=0;//拉低时钟开始数据传输
	 for(t=0;t<8;t++)
    {  
			Delay(); 
        I2C_SDA=(txd&0x80)>>7;//判断最高位为1或0；0写数据；1读数据
        txd<<=1; 	  
		Delay();

		I2C_SCL=1;
		Delay();Delay();

		I2C_SCL=0;	
		
    }	 
	
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
u8 I2C_ReadByte(unsigned char ack)
{
  unsigned char i,receive=0;
  SDA_IN();//SDA设置为输入
	Delay();  //
	
     for(i=0;i<8;i++ )
	{
		
        I2C_SCL=0; 
	
        Delay();
		
		I2C_SCL=1;
		Delay();  //
	
        receive<<=1;
		Delay();  //
        if(READ_SDA)
				{
				receive++;   
				}
				
		Delay();
   }					 
    if (ack)
        I2C_Ack(); //发送ACK 
    else
        I2C_NAck();//发送nACK  
    return receive;
}

//////////////////////////////////////////////////////////////
// 将多个字节写入指定设备 指定寄存器
u8 I2C_WriteBytes(u8 dev, u8 reg, u8 length, u8* data)
{
  u8 count = 0;
	I2C_Start();
	I2C_SendByte(dev);	   //发送写命令
	I2C_WaitAck();
	I2C_SendByte(reg);   //发送地址
  I2C_WaitAck();	  
	for(count=0;count<length;count++){
		I2C_SendByte(data[count]); 
		I2C_WaitAck(); 
	 }
	I2C_Stop();//产生一个停止条件

    return 1; //status == 0;
}
//  写入指定设备 指定寄存器一个字节
unsigned char I2C_WriteOneByte(unsigned char dev, unsigned char reg, unsigned char data)
{
   return I2C_WriteBytes(dev, reg, 1, &data);
}
///////////////////////////////////////////////////

uint8_t I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
  uint8_t res=0;
	
	I2C_Start();	
	I2C_SendByte(I2C_Addr);	   //发送写命令
	res++;
	I2C_WaitAck();
	I2C_SendByte(addr); 
	res++;  //发送地址
	I2C_WaitAck();	  
	//IIC_Stop();//产生一个停止条件	
	I2C_Start();
	I2C_SendByte(I2C_Addr+1); res++;          //进入接收模式			   
	I2C_WaitAck();
	res=I2C_ReadByte(0);	   
    I2C_Stop();//产生一个停止条件

	return res;
}
/**************************实现函数********************************************
*函数原型:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
void I2C_readBytes(u8 dev, u8 reg, u8 length, uint8_t *data){
    u8 count = 0;
	
	I2C_Start();
	I2C_SendByte(dev);	   //发送写命令   //发送写命令
	I2C_WaitAck();
	I2C_SendByte(reg);   //发送寄存器地址
  I2C_WaitAck();	  
	I2C_Start();
	I2C_SendByte(dev+1); //进入接收模式	
	I2C_WaitAck();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=I2C_ReadByte(1);  //带ACK的读数据
		 	else  data[count]=I2C_ReadByte(0); //最后一个字节NACK
	}
    I2C_Stop();//产生一个停止条件
  //  return count;
}


int GetData(char REG_Address)
{
char H,L;
H=I2C_ReadOneByte(0xD0,REG_Address);
L=I2C_ReadOneByte(0xD0,REG_Address+1);
return (H<<8)+L;   
}


