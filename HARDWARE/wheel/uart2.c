#include "uart2.h"
#include "msg.h"

UART4_MSG_S g_stUart2Msg;


/*驱动轮连接的是PC10 PC11接口，是UART4
  这个函数是初始化相关的USART设置*/
void UART2_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //UART4 TX；
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出；
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); //端口C；
        
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //UART4 RX；
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入；
    GPIO_Init(GPIOA, &GPIO_InitStructure); //端口C；

    USART_InitStructure.USART_BaudRate = WHEEL_ADP_USART_BAUD_RATE; //波特率；
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据位8位；
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //停止位1位；
    USART_InitStructure.USART_Parity = USART_Parity_No ; //无校验位；
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //无硬件流控；
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //收发模式；
    USART_Init(USART2, &USART_InitStructure);//配置串口参数；

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //设置中断组，4位抢占优先级，4位响应优先级；

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //中断号；
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7; //抢占优先级；
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //响应优先级；
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE); //使能串口；

}

void UART2_Init()
{
    UART2_Configuration();
}


void UART2_Send_Bytes(u8 *Data, u8 len) //发送；
{
    int i;

    for(i = 0; i < len; i++)
    {
        USART_SendData(USART2,Data[i]);
        while( USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET );
    }
}

//结束标记0d 0a
void UART2_IRQHandler(void) //中断处理函数；
{    
    if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET) //判断是否发生中断；
    {
        USART_ReceiveData(USART2);

        USART_SendData(USART2, 96);
        while( USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET );
                
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);         //清除中断标志
    }  
} 



