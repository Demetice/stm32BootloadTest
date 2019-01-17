#ifndef __UART4_H__
#define __UART4_H__

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "log.h"
#include "FreeRTOS.h"

#define WHEEL_ADP_USART_BAUD_RATE 57600
#define UART4_REC_LEN 256

typedef void (*P_FUNC_HANDLE_UART_RECEIVE_DATA)(u8* data, u8 len);

//function define
extern void UART4_Init(void);
extern void UART4_Send_Bytes(u8 *Data, u8 len);
extern void UART4_GetReceiveData(u16 *plen, u8 *data);


#endif

