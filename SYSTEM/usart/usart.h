#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 


#define USART_REC_LEN  			256  	//定义最大接收字节数 256
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收


typedef struct tagUartMsg
{
    u32 len;
    u8 buf[USART_REC_LEN];
}UART_MSG_S;


	  	
extern UART_MSG_S g_stUart1Msg;
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
#endif


