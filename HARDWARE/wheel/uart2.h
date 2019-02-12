#ifndef __UART2_H__
#define __UART2_H__

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "log.h"
#include "FreeRTOS.h"
#include "uart4.h"

//function define
extern void UART2_Init(void);
extern void UART2_Send_Bytes(u8 *Data, u8 len);
extern void UART2_GetReceiveData(u16 *plen, u8 *data);

#endif

