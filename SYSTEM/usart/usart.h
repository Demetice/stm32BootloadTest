#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 


#define USART_REC_LEN  			256  	//�����������ֽ��� 256
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����


typedef struct tagUartMsg
{
    u32 len;
    u8 buf[USART_REC_LEN];
}UART_MSG_S;


	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
#endif


