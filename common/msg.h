#ifndef _MSG_H_
#define _MSG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define uint8_t unsigned char
#define int32_t int
#define uint16_t unsigned short
#define uint32_t unsigned long


/**
  * - task and interupt message enum list.
  *  - Message enum struct, all message used by tasks and interupt must be defined here.
  */
typedef enum
{
    MSG_ID_NULL = 0,

    MSG_ID_USART1_DMA_RECEIVE,
    MSG_ID_WHEEL_STATE,
    MSG_ID_RED_LED_CONTROL,
    MSG_ID_MOTOR_SPEED,

    //here add your msg id


    //last msg, don't add under it
    MESSAGE_LIST_END /** max message count */
} MESSAGE_E;

/**
  * parameter of message struct
  * - 1. *pointer: point a message parameter which is greater than 4 bytes.
  * - 2. value: if your message parameter less than 4 bytes, assign the parameter to the value directly.
  *
  * @brief
  *      if the parameter is greater than 4 bytes, you should define a struct.  About the struct,
  *      please refer to software architecture files.
  */
typedef union
{
    void * pointer; /** transmit pointer */
    uint32_t value; /** transmit value   */

} PARAMETER_U;

/**
  * - message_id : identification of message, one of the enum struct: MESSAGE_E
  * - PARAMETER_U: parameter union struct, which can be a uint32_t value, or a pointer that point
  *                a parameter value struct.
  */
typedef struct
{
    int32_t message_id;
    PARAMETER_U parameter;

} MESSAGE_T;


struct QueueLinkNode
{
    QueueHandle_t queue;

    struct QueueLinkNode * next_queue;
};

struct QueueList_t
{
    struct QueueLinkNode * queue;
};

#define MESSAGE_IS_VALUE     (1 << 30)
#define MESSAGE_IS_POINTER   (0 << 30)
//#define MESSAGE_MAX_LENGTH   (256    )
#define MESSAGE_MAX_LENGTH   (256*3 )


int MessageSendFromISR(int send_msg, uint32_t para_pointer, BaseType_t *pxHigherPriorityTaskWoken);
int MessageSendTimeout(int send_msg, void * para_pointer, uint32_t para_length, uint32_t pointer_or_value, TickType_t 
xTicksToWait);
int MessageSend( int send_msg, void * para_pointer, uint32_t para_length, uint32_t pointer_or_value );
int MessageRecv(QueueHandle_t queue, int * const received_msg, void * received_msg_data, uint32_t * src_buffer_len, 
uint32_t dis_buffer_len);
int MessageRecvTimeout(QueueHandle_t queue, int *const received_msg, void *received_msg_data, uint32_t ms, uint32_t * 
src_buffer_len, uint32_t dis_buffer_len);
int message_queue_map(int msg, QueueHandle_t * queue);
int message_queue_unmap(int msg, QueueHandle_t queue);

int LinkListInit(void);

#ifdef __cplusplus
}
#endif

#endif

