#ifndef __UART_H
#define __UART_H

#include "main.h"


#define USART_REC_LEN               200         /* 定义最大接收字节数 200 */
#define RXBUFFERSIZE   1                        /* 缓存大小 */

extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 */
extern uint16_t g_usart_rx_sta;                 /* 接收状态标记 */
extern uint8_t g_uasrt1_rx[RXBUFFERSIZE];       /* HAL库USART接收Buffer */

extern uint8_t g_usart3_rx[1];                  //串口3接收数据
extern uint8_t g_forw,g_back,g_left,g_right;    //小车状态控制
extern uint8_t g_bluetooth_data;                //蓝牙数据缓存

void usart1_receive_callback(void);
void usart3_receive_callback(void);


#endif
