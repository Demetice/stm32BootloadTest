#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "log.h"

#define WHEEL_ADP_USART_BAUD_RATE 57600
#define USART_REC_LEN 256

static u8 g_aucUsartRxBuf[USART_REC_LEN];     
static u16 g_usUsartRxState=0;     
static u8 g_ucUartRxLen = 0;

/*驱动轮连接的是PC10 PC11接口，是UART4
  这个函数是初始化相关的USART设置*/
void UART4_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE );
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); //使能DMA传输

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //UART4 TX；
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出；
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure); //端口C；
        
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //UART4 RX；
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入；
    GPIO_Init(GPIOC, &GPIO_InitStructure); //端口C；

    USART_InitStructure.USART_BaudRate = 57600; //波特率；
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据位8位；
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //停止位1位；
    USART_InitStructure.USART_Parity = USART_Parity_No ; //无校验位；
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //无硬件流控；
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //收发模式；
    USART_Init(UART4, &USART_InitStructure);//配置串口参数；

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断组，4位抢占优先级，4位响应优先级；

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; //中断号；
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级；
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //响应优先级；
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(UART4, ENABLE); //使能串口；

      //相应的DMA配置
    DMA_DeInit(DMA2_Channel3);   //将DMA的通道5寄存器重设为缺省值  串口1对应的是DMA通道5
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;  //DMA外设ADC基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)g_aucUsartRxBuf;  //DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存
    DMA_InitStructure.DMA_BufferSize = USART_REC_LEN;  //DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
    DMA_Init(DMA2_Channel3, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道
    DMA_ClearFlag(DMA2_FLAG_GL3);
    DMA_Cmd(DMA2_Channel3, ENABLE);  //正式驱动DMA传输    
}

void UART4_Send_Byte(u8 Data) //发送一个字节；
{
    USART_SendData(UART4,Data);
    while( USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET );
}

void UART4_Send_Bytes(u8 *Data, u8 len) //发送字符串；
{
    int i;

    for(i = 0; i < len; i++)
    {
        UART4_Send_Byte(Data[i]);
    }
}

void UART4DmaClr(void)
{
    DMA_Cmd(DMA2_Channel3, DISABLE);
    DMA_SetCurrDataCounter(DMA2_Channel3, USART_REC_LEN);
    DMA_ClearFlag(DMA2_FLAG_GL3);
    DMA_Cmd(DMA2_Channel3, ENABLE);
}

//结束标记0d 0a
void UART4_IRQHandler(void) //中断处理函数；
{    
    if(USART_GetITStatus(UART4, USART_IT_IDLE) == SET) //判断是否发生中断；
    {
        USART_ReceiveData(UART4);
        g_ucUartRxLen = USART_REC_LEN - DMA_GetCurrDataCounter(DMA2_Channel3); //算出接本帧数据长度
        LOGD("receive data by dma len:%u", g_ucUartRxLen);
        UART4_Send_Bytes(g_aucUsartRxBuf, g_ucUartRxLen);

        USART_ClearITPendingBit(UART4, USART_IT_IDLE);         //清除中断标志
        UART4DmaClr();                   //恢复DMA指针，等待下一次的接收
    }  
} 


