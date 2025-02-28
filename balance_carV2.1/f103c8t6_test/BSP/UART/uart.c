#include "uart.h"
#include "usart.h"
#include "stdio.h"


/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");  /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* MDK下需要重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);     /* 等待上一个字符发送完成 */

    USART1->DR = (uint8_t)ch;             /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif
/******************************************************************************************/


/* 接收缓冲, 最大USART_REC_LEN个字节. */
uint8_t g_usart_rx_buf[USART_REC_LEN];	//代码规范变量命名：g开头表示全局变量，p开头表示指针变量

/*  接收状态
 *  bit15，      接收完成标志，收到换行\r 0x0A
 *  bit14，      接收到回车\n 0x0d
 *  bit13~0，    接收到的有效字节数目，最多2的14次方
*/
uint16_t g_usart_rx_sta = 0;

uint8_t g_uasrt1_rx[RXBUFFERSIZE];  /* HAL库使用的串口接收缓冲 */

uint8_t g_usart3_rx[1];                 //串口3接收数据
uint8_t g_forw,g_back,g_left,g_right;   //小车状态控制
uint8_t g_bluetooth_data;               //蓝牙数据缓存

/**
 * @brief       串口3数据接收回调函数
                数据处理在这里进行
 * @retval      无
 */
void usart3_receive_callback(void)
{
    g_bluetooth_data = g_usart3_rx[0];
    if(g_bluetooth_data == 'C')      g_forw=0,g_back=0,g_left=0,g_right=0;//刹
    else if(g_bluetooth_data == 'A') g_forw=1,g_back=0,g_left=0,g_right=0;//前
    else if(g_bluetooth_data == 'B') g_forw=0,g_back=1,g_left=0,g_right=0;//后
    else if(g_bluetooth_data == 'D') g_forw=0,g_back=0,g_left=0,g_right=1;//右
    else if(g_bluetooth_data == 'E') g_forw=0,g_back=0,g_left=1,g_right=0;//左
    else                             g_forw=0,g_back=0,g_left=0,g_right=0;//刹
    HAL_UART_Receive_IT(&huart3,g_usart3_rx,1);
    
}


/**
 * @brief       串口1数据接收回调函数
                数据处理在这里进行
 * @retval      无
 */
void usart1_receive_callback(void)
{
    if ((g_usart_rx_sta & 0x8000) == 0)             /* 判断位15，接收未完成 */
        {
            if (g_usart_rx_sta & 0x4000)                /* 判断位14 接收到了0x0d（即回车键） */
            {
                if (g_uasrt1_rx[0] != 0x0a)             /* 接收到的不是0x0a（即不是换行键） */
                {
                    g_usart_rx_sta = 0;                 /* 接收错误,重新开始 */
                }
                else                                    /* 接收到的是0x0a（即换行键） */
                {
                    g_usart_rx_sta |= 0x8000;           /* 接收完成了 */
                }
            }
            else                                        /* 还没收到0X0d（即回车键） */
            {
                if (g_uasrt1_rx[0] == 0x0d)             //收到回车
                    g_usart_rx_sta |= 0x4000;           //置回车标志位
                else                                    //正常接收，并统计字节数
                {
                    g_usart_rx_buf[g_usart_rx_sta & 0X3FFF] = g_uasrt1_rx[0]; //把接收的数据存入数组
                    g_usart_rx_sta++;                   //计数加1

                    if (g_usart_rx_sta > (USART_REC_LEN - 1))   //超过接收缓冲的长度
                    {
                        g_usart_rx_sta = 0;             /* 接收数据错误,重新开始接收 */
                    }
                }
            }
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)g_uasrt1_rx, RXBUFFERSIZE);//清除中断标志位,打开中断继续接收
}







/**
 * @brief       串口数据接收回调函数
                数据处理在这里进行
 * @param       huart:串口句柄
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)                    /* 如果是串口1 */
    {
        usart1_receive_callback();
    }
    else if (huart->Instance == USART3)               /* 如果是串口3 */
    {
        usart3_receive_callback();
    }
    
}



