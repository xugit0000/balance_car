/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
//#include "bmp.h"
#include "stdio.h"
#include "delay.h"
#include "mpu6050.h"//MPU6050驱动库
#include "inv_mpu.h"//陀螺仪驱动库
#include "inv_mpu_dmp_motion_driver.h" //DMP姿态解读库
#include "sr04.h"
#include "motor.h"
#include "encoder.h"
#include "uart.h"
#include "control.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t g_oledstring[50];   //OLED显示


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_ColorTurn(0);//0正常显示，1 反色显示
  OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
  
  MPU_Init();                                      //初始化MPU6050
  while(mpu_dmp_init())                            //初始化mpu_dmp库
  {
    OLED_ShowString(0,0,"MPU6050 Failed",8,1);   //显示字符串
    OLED_Refresh();                              //刷新显存
    delay_ms(50);                                //LED闪烁指示
//    printf("Initialization failed！\r\n");       //串口初始化失败上报
    OLED_ShowString(0,0,"              ",8,1);   //显示字符串
    OLED_Refresh();                              //刷新显存
  }
  OLED_Clear();
//  printf("Initialization successed！\r\n");      //串口初始化成功上报
  OLED_ShowString(0,0,"MPU6050 OK!",8,1);        //显示字符串
  OLED_Refresh();                                //刷新显存
//  OLED_Clear();
  
  set_motor1_enable();
  set_motor2_enable();
  
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//开启定时器2 编码器模式
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//开启定时器4 编码器模式
  
//  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);     //完成陀螺仪初始化后再开启中断，防止出错
//  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  HAL_UART_Receive_IT(&huart1, (uint8_t *)g_uasrt1_rx, RXBUFFERSIZE);//清除中断标志位,打开串口1中断继续接收
  HAL_UART_Receive_IT(&huart3,g_usart3_rx,1);//清除中断标志位,打开串口3中断继续接收
  
  
  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      
      
      /* 蓝牙测试 */
//      OLED_ShowString(80,0,g_usart3_rx,12,1);
      /* 编码器测试 */
//      sprintf((char *)g_oledstring,"encoder2:%5d ",g_encoder_left);
//      OLED_ShowString(0,36,(uint8_t *)g_oledstring,12,1);
//      sprintf((char *)g_oledstring,"encoder4:%5d ",g_encoder_right);
//      OLED_ShowString(0,48,(uint8_t *)g_oledstring,12,1);
      
      /* 陀螺仪测试 */
//      mpu_dmp_get_data(&g_pitch,&g_roll,&g_yaw);        //得到角度值
//      MPU_Get_Accelerometer(&g_aacx,&g_aacy,&g_aacz);   //得到加速度传感器数据
//      MPU_Get_Gyroscope(&g_gyrox,&g_gyroy,&g_gyroz);    //得到陀螺仪数据
//      g_temp=MPU_Get_Temperature();                     //得到温度值
//      sprintf((char *)g_oledstring,"Pitch:%4.1f ",g_pitch);
//      OLED_ShowString(0,0,(uint8_t *)g_oledstring,12,1);
//      sprintf((char *)g_oledstring,"roll:%4.1f ",g_roll);
//      OLED_ShowString(0,12,(uint8_t *)g_oledstring,12,1);
//      sprintf((char *)g_oledstring,"yaw:%4.1f ",g_yaw);
//      OLED_ShowString(0,24,(uint8_t *)g_oledstring,12,1);
//      printf("\r\ntemp:%.1f\r\n",(float)g_temp/100);//读取温度值扩大了一百倍
//      printf("pitch:%.1f,   roll:%.1f,   yaw:%.1f\r\n",g_pitch,g_roll,g_yaw);//上报角度数据
//      printf("gyrox:%d,gyroy:%d,gyroz:%d,aacx:%d,aacy:%d,aacz:%d\r\n",g_gyrox,g_gyroy,g_gyroz,g_aacx,g_aacy,g_aacz);//上报角速度数据，角加速度数据
      
      /* 超声波测试 */
//      get_distance();
//      sprintf((char *)g_oledstring,"dis:%4.1f cm",g_distance);
//      OLED_ShowString(0,36,(uint8_t *)g_oledstring,12,1);
//      OLED_Refresh();
//      
//      HAL_Delay(10);
//      HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
