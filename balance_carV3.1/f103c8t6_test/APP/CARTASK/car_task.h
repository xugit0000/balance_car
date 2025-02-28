#ifndef __CAR_TASK_H
#define __CAR_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/*FreeRTOS配置*/
//优先级数值越高越优先

/* control_task 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
extern TaskHandle_t g_control_task_handle;         //任务句柄
#define CONTROL_TASK_PRIO             5            //任务优先级
#define CONTROL_TASK_STACK_SIZE      256           //任务堆栈大小
void control_task(void *pvParameters);             //任务函数声明

/* display_task 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
extern TaskHandle_t g_display_task_handle;         //任务句柄
#define DISPLAY_TASK_PRIO             4            //任务优先级
#define DISPLAY_TASK_STACK_SIZE      256           //任务堆栈大小
void display_task(void *pvParameters);             //任务函数声明

/* led_task 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
extern TaskHandle_t g_led_task_handle;             //任务句柄
#define LED_TASK_PRIO                 3            //任务优先级
#define LED_TASK_STACK_SIZE          128           //任务堆栈大小
void led_task(void *pvParameters);                 //任务函数声明

/* 信号量 */
extern SemaphoreHandle_t g_binsemphore_mpu6050;    //mpu6050二值信号量句柄



#endif
