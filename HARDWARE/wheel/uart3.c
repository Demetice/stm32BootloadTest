#include "uart3.h"
#include "msg.h"
#include "led.h"

UART4_MSG_S g_stUart3Msg;


/*���������ӵ���PC10 PC11�ӿڣ���UART4
  ��������ǳ�ʼ����ص�USART����*/
void UART3_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);          
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //ʹ��DMA����

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);

    USART_InitStructure.USART_BaudRate = WHEEL_ADP_USART_BAUD_RATE; //�����ʣ�
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //����λ8λ��
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //ֹͣλ1λ��
    USART_InitStructure.USART_Parity = USART_Parity_No ; //��У��λ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //��Ӳ�����أ�
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //�շ�ģʽ��
    USART_Init(USART3, &USART_InitStructure);//���ô��ڲ�����

    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //�����ж��飬4λ��ռ���ȼ���4λ��Ӧ���ȼ���

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //�жϺţ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ���
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //��Ӧ���ȼ���
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART3, ENABLE); //ʹ�ܴ��ڣ�

      //��Ӧ��DMA����
    DMA_DeInit(DMA1_Channel3);   //��DMA��ͨ��5�Ĵ�������Ϊȱʡֵ  ����1��Ӧ����DMAͨ��5
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;  //DMA����ADC����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)g_stUart3Msg.buf;  //DMA�ڴ����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //���ݴ��䷽�򣬴������ȡ���͵��ڴ�
    DMA_InitStructure.DMA_BufferSize = UART4_REC_LEN;  //DMAͨ����DMA����Ĵ�С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��
    DMA_ClearFlag(DMA1_FLAG_GL3);
    DMA_Cmd(DMA1_Channel3, ENABLE);  //��ʽ����DMA����  
}

void UART3_Init()
{
    UART3_Configuration();
}


void UART3_Send_Bytes(u8 *Data, u8 len) //���ͣ�
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

//�������0d 0a
void UART3_IRQHandler(void) //�жϴ�������
{    
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if(USART_GetITStatus(USART3, USART_IT_IDLE) == SET) //�ж��Ƿ����жϣ�
    {
        USART_ReceiveData(USART3);
        g_stUart3Msg.len = UART4_REC_LEN - DMA_GetCurrDataCounter(DMA1_Channel3); //����ӱ�֡���ݳ���

        //�ͷ����ݽ�������źţ������ж����洦�����ݽ���
        MessageSendFromISR(MSG_ID_WHEEL_STATE, (uint32_t)&g_stUart3Msg, &xHigherPriorityTaskWoken);
        UART3_Send_Bytes(g_stUart3Msg.buf, g_stUart3Msg.len);
        //LED0=~LED0;
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);         //����жϱ�־
        UART3DmaClr();                   //�ָ�DMAָ�룬�ȴ���һ�εĽ���
    }  
} 





