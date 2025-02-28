#include "car_task.h"
#include "stdio.h"
#include "control.h"
#include "mpu6050.h"
#include "oled.h"


/* 任务句柄 */
TaskHandle_t g_control_task_handle;        //control_task任务句柄
TaskHandle_t g_display_task_handle;        //display_task任务句柄
TaskHandle_t g_led_task_handle;            //led_task任务句柄

/* 信号量 */
SemaphoreHandle_t g_binsemphore_mpu6050;          //mpu6050二值信号量句柄


/* 任务函数 */
/* /control_task任务函数 */
void control_task(void *pvParameters)
{
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);     //完成陀螺仪初始化后再开启中断，防止出错
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    while(1)
    {
        xSemaphoreTake(g_binsemphore_mpu6050,portMAX_DELAY);        //等待陀螺仪10ms中断发送一次信号量
        car_control();
//        HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
    }
    
}

/* display_task任务函数 */
void display_task(void *pvParameters)
{
    
    while(1)
    {
        /* 陀螺仪数据 */
        sprintf((char *)g_oledstring,"Pitch:%4.1f ",g_pitch);
        OLED_ShowString(0,0,(uint8_t *)g_oledstring,8,1);
        sprintf((char *)g_oledstring,"roll:%4.1f ",g_roll);
        OLED_ShowString(0,8,(uint8_t *)g_oledstring,8,1);
        sprintf((char *)g_oledstring,"yaw:%4.1f ",g_yaw);
        OLED_ShowString(0,16,(uint8_t *)g_oledstring,8,1);
        OLED_Refresh();
        /* 编码器数据 */
        sprintf((char *)g_oledstring,"encoder2:%5d ",g_encoder_left);
        OLED_ShowString(0,24,(uint8_t *)g_oledstring,8,1);
        sprintf((char *)g_oledstring,"encoder4:%5d ",g_encoder_right);
        OLED_ShowString(0,32,(uint8_t *)g_oledstring,8,1);
        
        vTaskDelay(100);
    }
}

/* led_task任务函数 */
void led_task(void *pvParameters)
{
    
    while(1)
    {
        HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
        vTaskDelay(500);
    }
}
