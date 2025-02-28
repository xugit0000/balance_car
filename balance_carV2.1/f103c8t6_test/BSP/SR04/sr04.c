#include "sr04.h"
#include "tim.h"
#include "delay.h"


uint16_t g_sr04_time;           //超声波计时
float g_distance;               //超声波距离,cm


/* 触发超声波 */
void get_distance(void)
{
    HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_RESET);
}


/* 超声波中断回调函数 */
void sr04_callback(void)
{
    if(HAL_GPIO_ReadPin(ECHO_GPIO_Port,ECHO_Pin) == GPIO_PIN_SET)
        {
            __HAL_TIM_SetCounter(&htim3,0);
            HAL_TIM_Base_Start(&htim3);
        }
        else
        {
            HAL_TIM_Base_Stop(&htim3);
            g_sr04_time = __HAL_TIM_GetCounter(&htim3);
            g_distance = g_sr04_time * 0.017;       //time/1000000*340*100/2
        }
}






