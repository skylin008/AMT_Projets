#include "I2C.h"


//��ʼ����PB8,PB9ģ��I2C������ģʽ
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void I2C_Config(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/

void I2C_Delay(uint16_t i)
{
	//uint32_t i;
	for(;i>0;i--);		/*1T��Ƭ����12MHz������ʱʱi=50���ȶ��ز���DRS1000
							������ø����ٵĵ�Ƭ����Ҫ�Ӵ�ѭ��������*/
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
//I2C��ʼ�ź�
int I2C_Start(void)
{
	I2C_SCL=0;
	SDA_OUT();     //sda�����
  I2C_SDA=1;
	Delay();
	
	if(!READ_SDA)return 0;	
	I2C_SCL=1;
	Delay();Delay();Delay();
	
	I2C_SDA=0;//START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	Delay();Delay();
	
	I2C_SCL=0;  //ǯסI2C���ߣ�׼�����ͻ�������� 
	Delay();
	
	return 1;
}
//I2Cֹͣ�ź�
void I2C_Stop(void)
{
	SDA_OUT();//sda�����
	Delay();
  I2C_SCL=0;
	I2C_SDA=0;
	Delay();
	I2C_SCL=1;
	Delay();//Delay();Delay();
	
	I2C_SDA=1;//����I2C���߽����ź�
	Delay();Delay();
	I2C_SCL=0;
	//I2C_SCL_0();
}
//����Ӧ���ź�
//SCLΪ��ʱ��SDA������ΪӦ��SDA������Ϊ��Ӧ��
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

	I2C_SDA=1;   //CPU�ͷ�SDA
}
//������Ӧ���ź�
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
//�ȴ�Ӧ���ź�
int I2C_WaitAck(void)
{
  u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	Delay();
	I2C_SDA=1;           //CPU�ͷ�SDA����
	Delay();

	I2C_SCL=1;           //CPU����SCL��ƽ������һ��ʱ�ӣ���ʱ�᷵��ACKӦ��
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
	I2C_SCL=0;   //ʱ�����0 	   
Delay();	
	return 1;
}






///////////////////////////////////////////////////////////////////////
//I2C����һ���ֽ�
void I2C_SendByte(u8 txd)
{
  u8 t;   
	SDA_OUT(); 	 

//	I2C_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	 for(t=0;t<8;t++)
    {  
			Delay(); 
        I2C_SDA=(txd&0x80)>>7;//�ж����λΪ1��0��0д���ݣ�1������
        txd<<=1; 	  
		Delay();

		I2C_SCL=1;
		Delay();Delay();

		I2C_SCL=0;	
		
    }	 
	
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
u8 I2C_ReadByte(unsigned char ack)
{
  unsigned char i,receive=0;
  SDA_IN();//SDA����Ϊ����
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
        I2C_Ack(); //����ACK 
    else
        I2C_NAck();//����nACK  
    return receive;
}

//////////////////////////////////////////////////////////////
// ������ֽ�д��ָ���豸 ָ���Ĵ���
u8 I2C_WriteBytes(u8 dev, u8 reg, u8 length, u8* data)
{
  u8 count = 0;
	I2C_Start();
	I2C_SendByte(dev);	   //����д����
	I2C_WaitAck();
	I2C_SendByte(reg);   //���͵�ַ
  I2C_WaitAck();	  
	for(count=0;count<length;count++){
		I2C_SendByte(data[count]); 
		I2C_WaitAck(); 
	 }
	I2C_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
}
//  д��ָ���豸 ָ���Ĵ���һ���ֽ�
unsigned char I2C_WriteOneByte(unsigned char dev, unsigned char reg, unsigned char data)
{
   return I2C_WriteBytes(dev, reg, 1, &data);
}
///////////////////////////////////////////////////

uint8_t I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
  uint8_t res=0;
	
	I2C_Start();	
	I2C_SendByte(I2C_Addr);	   //����д����
	res++;
	I2C_WaitAck();
	I2C_SendByte(addr); 
	res++;  //���͵�ַ
	I2C_WaitAck();	  
	//IIC_Stop();//����һ��ֹͣ����	
	I2C_Start();
	I2C_SendByte(I2C_Addr+1); res++;          //�������ģʽ			   
	I2C_WaitAck();
	res=I2C_ReadByte(0);	   
    I2C_Stop();//����һ��ֹͣ����

	return res;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
void I2C_readBytes(u8 dev, u8 reg, u8 length, uint8_t *data){
    u8 count = 0;
	
	I2C_Start();
	I2C_SendByte(dev);	   //����д����   //����д����
	I2C_WaitAck();
	I2C_SendByte(reg);   //���ͼĴ�����ַ
  I2C_WaitAck();	  
	I2C_Start();
	I2C_SendByte(dev+1); //�������ģʽ	
	I2C_WaitAck();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=I2C_ReadByte(1);  //��ACK�Ķ�����
		 	else  data[count]=I2C_ReadByte(0); //���һ���ֽ�NACK
	}
    I2C_Stop();//����һ��ֹͣ����
  //  return count;
}


int GetData(char REG_Address)
{
char H,L;
H=I2C_ReadOneByte(0xD0,REG_Address);
L=I2C_ReadOneByte(0xD0,REG_Address+1);
return (H<<8)+L;   
}


