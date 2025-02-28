/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "semphr.h"
#include "delay.h"
#include "car_task.h"
#include "oled.h"
#include "stdio.h"
#include "delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char pcWriteBuffer[300];        //打印信息数组

/* USER CODE END Variables */
/* Definitions for start_task */
osThreadId_t start_taskHandle;
const osThreadAttr_t start_task_attributes = {
  .name = "start_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
    vTaskList(pcWriteBuffer);
    printf("\r\n任务名\t\t 状态\t 优先级\t 剩余栈\t 任务序号\r\n");
    printf("%s\r\n",pcWriteBuffer);
    
//    vTaskGetRunTimeStats(pcWriteBuffer);
//    printf("任务名\t\t 运行时间\t 运行所占百分比\r\n");
//    printf("%s\r\n", pcWriteBuffer);
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  
    
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of start_task */
  start_taskHandle = osThreadNew(StartDefaultTask, NULL, &start_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the start_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    BaseType_t err;
    
    /* 创建二值信号量 */
    g_binsemphore_mpu6050 = xSemaphoreCreateBinary();       //创建mpu6050二值信号量
    
    
    /* 进入临界区 */
    taskENTER_CRITICAL();    //关闭任务调度中断，等创建完所有任务退出临界区再开始调度任务，否则创建任务后会立即就绪执行，与优先级不符
    
    /* 创建任务1 */
    err = xTaskCreate((TaskFunction_t         ) control_task,              //任务函数
                     (char *                 ) "control_task",             //任务名称
                     (configSTACK_DEPTH_TYPE ) CONTROL_TASK_STACK_SIZE,    //任务堆栈大小
                     (void *                 ) NULL,                       //传入给任务函数的参数
                     (UBaseType_t            ) CONTROL_TASK_PRIO,          //任务优先级
                     (TaskHandle_t *         ) &g_control_task_handle );   //任务句柄
    if(err == pdPASS)
    {
        OLED_ShowString(0,8,"CONTROL_TASK OK!",8,1);    //显示字符串
        OLED_Refresh();                                 //刷新显存
    }
    else
    {
        OLED_ShowString(0,8,"CONTROL_TASK ERR!",8,1);   //显示字符串
        OLED_Refresh();                                 //刷新显存
    }
    /* 创建任务2 */
    err = xTaskCreate((TaskFunction_t         ) display_task,              //任务函数
                     (char *                 ) "display_task",             //任务名称
                     (configSTACK_DEPTH_TYPE ) DISPLAY_TASK_STACK_SIZE,    //任务堆栈大小
                     (void *                 ) NULL,                       //传入给任务函数的参数
                     (UBaseType_t            ) DISPLAY_TASK_PRIO,          //任务优先级
                     (TaskHandle_t *         ) &g_display_task_handle );   //任务句柄
    if(err == pdPASS)
    {
        OLED_ShowString(0,16,"DISPLAY_TASK OK!",8,1);     //显示字符串
        OLED_Refresh();                                   //刷新显存
    }
    else
    {
        OLED_ShowString(0,16,"DISPLAY_TASK ERR!",8,1);    //显示字符串
        OLED_Refresh();                                   //刷新显存
    }
    /* 创建任务3 */
    err = xTaskCreate((TaskFunction_t         ) led_task,                  //任务函数
                     (char *                 ) "led_task",                 //任务名称
                     (configSTACK_DEPTH_TYPE ) LED_TASK_STACK_SIZE,        //任务堆栈大小
                     (void *                 ) NULL,                       //传入给任务函数的参数
                     (UBaseType_t            ) LED_TASK_PRIO,              //任务优先级
                     (TaskHandle_t *         ) &g_led_task_handle );       //任务句柄
    if(err == pdPASS)
    {
        OLED_ShowString(0,24,"LED_TASK OK!",8,1);       //显示字符串
        OLED_Refresh();                                 //刷新显存
    }
    else
    {
        OLED_ShowString(0,24,"LED_TASK ERR!",8,1);      //显示字符串
        OLED_Refresh();                                 //刷新显存
    }
    
    
    delay_ms(1000);
    OLED_Clear();
    
    /* 删除开始任务 */
    vTaskDelete(NULL);
                
    /* 退出临界区 */
    taskEXIT_CRITICAL();    //等待start_task执行完再开始任务调度，保证符合优先级
  
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

