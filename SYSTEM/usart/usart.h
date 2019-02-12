#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 


#define USART_REC_LEN  			2048  	//定义最大接收字节数 256
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define USART1_MSG_START_BYTE 0x80
#define USART1_MSG_END_BYTE 0x0a


//usart1 接收到的消息处理函数，供其它模块注册
typedef int (*P_FUN_USART1_MSG_HANDLE)(u8 *buf, u32 len);
typedef void (*P_FUN_USART1_MSG_REPLY)(void);

typedef struct tagUsart1MsgReg
{
    u32 cmd;
    P_FUN_USART1_MSG_HANDLE pfnHandle;
    P_FUN_USART1_MSG_REPLY pfnRepeat;
}UART1_MSG_REG_S;

typedef struct tagUartMsg
{
    u32 len;
    u8 buf[USART_REC_LEN];
}UART_MSG_S;

extern UART_MSG_S g_stUart1Msg;
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void USART1_Send_Bytes(u8 *Data, u8 len); //发送字符串；





#endif


