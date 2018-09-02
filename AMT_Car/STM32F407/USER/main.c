#include "init.h"
extern u8 Loop_400Hz_flag;
extern u8 Loop_200Hz_flag;
extern u8 Loop_100Hz_flag;
int main()
{
    init_All();
		
		while(1)
		{
			if(Loop_400Hz_flag)
			{
			  Loop_400Hz_flag=0;
				//
			
				Motor_SpeedTunner();
			
			//	update_Speed(Motor);
			//printf("%d\r\n",Motor[0].SpeedCur);
			}
			if(Loop_100Hz_flag)
			{
				Loop_100Hz_flag=0;
			#if 0
				
	//Ö¡Í·
		USART_SendByte(USART2,0x03);
		 USART_SendByte(USART2,0xFC);
         //Êý¾Ý	
	          	USART_Send2Byte(USART2,AMT_Car.Motor[0].SpeedCur);
				      USART_Send2Byte(USART2,AMT_Car.Motor[0].SpeedSet);
             
				

				
  //Ö¡Î²
		 USART_SendByte(USART2,0xFC);
		USART_SendByte(USART2,0x03);	
#endif
		
			}
		
		}
		
	
	
	
	  
}

