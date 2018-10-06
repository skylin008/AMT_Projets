
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "Copter_Param.h"
#include "Motor.h"
#include "Data_Exchange.h"
#include "MPU6050.h"
#include "IMU.h"
#include "LED.h"
#include "PID.h"

//飞行模式
#include "Stabilize.h"
#include "Headhold.h"



uint16_t ms_Count;   //主循环时基
uint8_t loop100Hz_flag,loop50Hz_flag,loop20Hz_flag,loop10Hz_flag,loop1Hz_flag;
uint16_t Frame;   //主循环频率
/**
  * @brief  
  * @param  
  * @retval None
  */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Print_Head(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Print_Head();  
  init_Motor();
  init_RX();
  init_MPU6050();
  HAL_TIM_Base_Start_IT(&htim2);                         //打开基本定时器中断更新
  
  init_PID();          //载入PID参数
  set_LED_State(Wait_armed);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
 
  static uint8_t armed=RESET;
 
  //检查解锁与上锁状态
  if(!armed&&Rx_Channel.Rx_Thr<Mincheck&&Rx_Channel.Rx_Yaw>Maxcheck)
     {armed=SET;set_LED_State(Armed);} 
  else if(armed&&Rx_Channel.Rx_Thr<Mincheck&&Rx_Channel.Rx_Yaw<Mincheck)
      {armed=RESET;set_LED_State(Wait_armed);}

  /*******************************************/
  static uint8_t runMotors=RESET;   //电机是否运行标志位
  if(armed&&Rx_Channel.Rx_Thr>Mincheck) runMotors=SET;
  else runMotors=RESET;
  if(1)
  {
    uint32_t now=Get_Tick();   //10us计数
    static uint32_t timer=0;
    float dt=(float)(now-timer)*0.00001;
    timer=now;
    
    //printf("%d\n",timer);
    update_IMU(dt);                    //更新IMU

    if(runMotors)       //电机处于转动状态
    {
      //遥控信号缩小10倍:-50~50
      float setpointRoll=(float)(Rx_Channel.Rx_Roll-1500)/10;
      float setpointPitch=(float)(Rx_Channel.Rx_Pitch-1500)/10;
      float setpointYaw=(float)(Rx_Channel.Rx_Yaw-1500)/10;

      //检查飞行模式
      check_FlyMode();
    
    if(Fly_Mode.Headhold&&fabs(setpointYaw)<0.5f)      //锁头模式，Yaw轴外环加P
    {
       setpointYaw=update_Headhold();    //更新计算setpointYaw
    }
    else reset_Headhold();     //Yaw有输入则不断更新Yaw期望值,setpointYaw即为期望角速度
    
    //外环
    setpointYaw*=stickScalingYaw;  //Yaw角外环

      if(Fly_Mode.Stabilize)   //自稳模式
      {
        uint8_t maxAngleInclination=maxAngle_Stable;    //自稳模式下最大倾角
        setpointRoll=constrain(setpointRoll,-maxAngleInclination,maxAngleInclination)-MPU6050.Axis_Angle.Roll;
        setpointPitch=constrain(setpointPitch,-maxAngleInclination,maxAngleInclination)-MPU6050.Axis_Angle.Pitch;
      }
      else {}   //否则为速率模式，无外环
      
      setpointRoll*=AngleKp;       //角度化为期望角速度
      setpointPitch*=AngleKp;      //角度化为期望角速度
      //printf("%d %d %d\n",(int16_t)setpointRoll,(int16_t)setpointPitch,(int16_t)setpointYaw);

      //内环
      float RollOut=updatePID(&pidRoll,setpointRoll,MPU6050.Axis_Rate.Roll,dt);
      float PitchOut=updatePID(&pidPitch,setpointPitch,MPU6050.Axis_Rate.Pitch,dt);
      float YawOut=updatePID(&pidYaw,setpointYaw,MPU6050.Axis_Rate.Yaw,dt);
      //printf("%d\n",(int16_t)pidRoll.iTerm);
      //printf("%d %d %d\n",(int16_t)RollOut,(int16_t)PitchOut,(int16_t)YawOut);
      //当前油门值
      float Thr=(float)(Rx_Channel.Rx_Thr-1000)/2;

      float Motor[4];
      for(uint8_t i=0;i<4;i++)
            Motor[i]=(float)Thr;

        //X模式下
            Motor[0]-=RollOut;
            Motor[1]+=RollOut;
            Motor[2]+=RollOut;
            Motor[3]-=RollOut;

            Motor[0]+=PitchOut;
            Motor[1]-=PitchOut;
            Motor[2]+=PitchOut;
            Motor[3]-=PitchOut;

            Motor[0]+=YawOut;
            Motor[1]+=YawOut;
            Motor[2]-=YawOut;
            Motor[3]-=YawOut;

        update_Motor(Motor);  //更新电机PWM
    }
    else {
      stop_Motor();               //停转电机
      resetPIDRollPitchYaw();     //重置PID参数

    }




  }
 
  

  Frame++;
  //10ms，发送上位机波形
  if(loop100Hz_flag==SET)
  {
    loop100Hz_flag=0;
    PC_Send_Data();
    if(loop1Hz_flag)
    {
      loop1Hz_flag=0;
      //printf("%d\n",Frame);
      Frame=0;

    }
  }
  
  //HAL_Delay(1);
    
  // static uint32_t looptimer;
  // while(Get_Tick()-looptimer<200) //循环控制在500Hz内
  // {}
  // looptimer=Get_Tick();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void Print_Head(void)
{
  printf("***************Welcome to AMT_Copter V1.0**************\n");
  printf("*******************Write By HamlinZheng****************\n");
  printf("**************Github:github.com/HamlinZheng************\n");
  printf("***********************Start Init**********************\n");
  set_LED_State(Initting);
}
//系统时钟1ms回调函数
void HAL_SYSTICK_Callback(void)
{
  ms_Count++;          //计数到2s，max为2000
  if(ms_Count%10==0)
  {
    check_LED(); //每10ms
    loop100Hz_flag=1;   //10ms
    if(ms_Count%20==0) loop50Hz_flag=1;  //20ms

    if(ms_Count%50==0) 
    {
      loop20Hz_flag=1;        //50ms
      
      if(ms_Count%100==0)
      {
        loop10Hz_flag=1;   //100ms
        if(ms_Count%1000==0) loop1Hz_flag=1;  //1s
      } 
    }
  } 

  if(ms_Count==2000) ms_Count=0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
