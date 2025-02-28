/**
 ****************************************************************************************************
 * @file        delay.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.1
 * @date        2023-02-25
 * @brief       使用SysTick的普通计数模式对延迟进行管理(支持ucosii)
 *              提供delay_init初始化函数， delay_us和delay_ms等延时函数
 * @license     Copyright (c) 2022-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20230206
 * 第一次发布
 * V1.1 20230225
 * 修改SYS_SUPPORT_OS部分代码, 默认仅支持UCOSII 2.93.01版本, 其他OS请参考实现
 * 修改delay_init不再使用8分频,全部统一使用MCU时钟
 * 修改delay_us使用时钟摘取法延时, 兼容OS
 * 修改delay_ms直接使用delay_us延时实现.
 *
 ****************************************************************************************************
 */

#include "delay.h"


//static uint32_t g_fac_us = 0;       /* us延时倍乘数 */



/**
 * @brief     初始化延迟函数
 * @param     sysclk: 系统时钟频率, 即CPU频率(rcc_c_ck), 72MHz
 * @retval    无
 */  
//void delay_init(uint16_t sysclk)
//{
//    g_fac_us = sysclk;                                  /* 由于在HAL_Init中已对systick做了配置，所以这里无需重新配置 */
//}

/**
 * @brief     延时nus
 * @note      无论是否使用OS, 都是用时钟摘取法来做us延时
 * @param     nus: 要延时的us数
 * @note      nus取值范围: 0 ~ (2^32 / fac_us) (fac_us一般等于系统主频, 自行套入计算)
 * @retval    无
 */
void delay_us(uint32_t nus)
{
    extern TIM_HandleTypeDef htim3;         //这里换成cubemx中为HAL库选的时基定时器
    TIM_HandleTypeDef *timebase = &htim3;
    
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = __HAL_TIM_GET_AUTORELOAD(timebase);        /* LOAD的值 */
    ticks = nus * reload / (1000);          /* 需要的节拍数 */

    told = __HAL_TIM_GET_COUNTER(timebase);  /* 刚进入时的计数器值 */
    while (1)
    {
        tnow = __HAL_TIM_GET_COUNTER(timebase);
        if (tnow != told)
        {
            if (tnow > told)
            {
                tcnt += tnow - told;        /* 这里注意一下SYSTICK是一个递减的计数器就可以了,注意普通定时器是递增的，所以要交换told,tnow */
            }
            else
            {
                tcnt += reload - told + tnow;       //注意普通定时器是递增的，所以要交换told,tnow
            }
            told = tnow;
            if (tcnt >= ticks) 
            {
                break;                      /* 时间超过/等于要延迟的时间,则退出 */
            }
        }
    }
}

/**
 * @brief     延时nms
 * @param     nms: 要延时的ms数 (0< nms <= (2^32 / fac_us / 1000))(fac_us一般等于系统主频, 自行套入计算)
 * @retval    无
 */
void delay_ms(uint16_t nms)
{
    for (int i = 0; i < nms; i++)
        delay_us(1000);
}

/**
 * @brief       HAL库内部函数用到的延时
 * @note        HAL库的延时默认用Systick，如果我们没有开Systick的中断会导致调用这个延时后无法退出
 * @param       Delay : 要延时的毫秒数
 * @retval      None
 */
//void HAL_Delay(uint32_t Delay)
//{
//     delay_ms(Delay);
//}


/**
 * @brief       获得系统时间ns
 * @param       无
 * @retval      系统时间ns
 */
uint64_t system_get_ns(void)
{
    //extern uint32_t HAL_GetTick(void);
    extern TIM_HandleTypeDef        htim3;
    TIM_HandleTypeDef *timebase = &htim3;
    
    uint64_t ns = HAL_GetTick();
    uint64_t cnt;
    uint64_t reload;

    cnt = __HAL_TIM_GET_COUNTER(timebase);
    reload = __HAL_TIM_GET_AUTORELOAD(timebase);

    ns *= 1000000;
    ns += cnt * 1000000 / reload;
    return ns;
}







