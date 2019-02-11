#include "uart3.h"
#include "msg.h"
#include "led.h"

UART4_MSG_S g_stUart3Msg;


/*驱动轮连接的是PC10 PC11接口，是UART4
  这个函数是初始化相关的USART设置*/
void UART3_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);          
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA传输

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);

    USART_InitStructure.USART_BaudRate = WHEEL_ADP_USART_BAUD_RATE; //波特率；
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据位8位；
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //停止位1位；
    USART_InitStructure.USART_Parity = USART_Parity_No ; //无校验位；
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //无硬件流控；
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //收发模式；
    USART_Init(USART3, &USART_InitStructure);//配置串口参数；

    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //设置中断组，4位抢占优先级，4位响应优先级；

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //中断号；
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级；
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //响应优先级；
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART3, ENABLE); //使能串口；

      //相应的DMA配置
    DMA_DeInit(DMA1_Channel3);   //将DMA的通道5寄存器重设为缺省值  串口1对应的是DMA通道5
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;  //DMA外设ADC基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)g_stUart3Msg.buf;  //DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存
    DMA_InitStructure.DMA_BufferSize = UART4_REC_LEN;  //DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道
    DMA_ClearFlag(DMA1_FLAG_GL3);
    DMA_Cmd(DMA1_Channel3, ENABLE);  //正式驱动DMA传输  
}

void UART3_Init()
{
    UART3_Configuration();
}


void UART3_Send_Bytes(u8 *Data, u8 len) //发送；
{
    int i;

    for(i = 0; i < len; i++)
    {
        USART_SendData(USART3,Data[i]);
        while( USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET );
    }
}

void UART3DmaClr(void)
{
    DMA_Cmd(DMA1_Channel3, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel3, UART4_REC_LEN);
    DMA_ClearFlag(DMA1_FLAG_GL3);
    DMA_Cmd(DMA1_Channel3, ENABLE);
}

//结束标记0d 0a
void UART3_IRQHandler(void) //中断处理函数；
{    
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if(USART_GetITStatus(USART3, USART_IT_IDLE) == SET) //判断是否发生中断；
    {
        USART_ReceiveData(USART3);
        g_stUart3Msg.len = UART4_REC_LEN - DMA_GetCurrDataCounter(DMA1_Channel3); //算出接本帧数据长度

        //释放数据接收完毕信号，不在中断里面处理数据解析
        MessageSendFromISR(MSG_ID_WHEEL_STATE, (uint32_t)&g_stUart3Msg, &xHigherPriorityTaskWoken);
        UART3_Send_Bytes(g_stUart3Msg.buf, g_stUart3Msg.len);
        //LED0=~LED0;
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);         //清除中断标志
        UART3DmaClr();                   //恢复DMA指针，等待下一次的接收
    }  
} 





