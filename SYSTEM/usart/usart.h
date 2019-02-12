#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 


#define USART_REC_LEN  			2048  	//�����������ֽ��� 256
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

#define USART1_MSG_START_BYTE 0x80
#define USART1_MSG_END_BYTE 0x0a


//usart1 ���յ�����Ϣ��������������ģ��ע��
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
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void USART1_Send_Bytes(u8 *Data, u8 len); //�����ַ�����





#endif


