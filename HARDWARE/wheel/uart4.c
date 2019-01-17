#include "uart4.h"
#include "msg.h"

UART4_MSG_S g_stUart4Msg;


/*���������ӵ���PC10 PC11�ӿڣ���UART4
  ��������ǳ�ʼ����ص�USART����*/
void UART4_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE );
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); //ʹ��DMA����

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //UART4 TX��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�������������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure); //�˿�C��
        
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //UART4 RX��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //�������룻
    GPIO_Init(GPIOC, &GPIO_InitStructure); //�˿�C��

    USART_InitStructure.USART_BaudRate = 57600; //�����ʣ�
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //����λ8λ��
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //ֹͣλ1λ��
    USART_InitStructure.USART_Parity = USART_Parity_No ; //��У��λ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //��Ӳ�����أ�
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //�շ�ģʽ��
    USART_Init(UART4, &USART_InitStructure);//���ô��ڲ�����

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //�����ж��飬4λ��ռ���ȼ���4λ��Ӧ���ȼ���

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; //�жϺţ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7; //��ռ���ȼ���
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //��Ӧ���ȼ���
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(UART4, ENABLE); //ʹ�ܴ��ڣ�

      //��Ӧ��DMA����
    DMA_DeInit(DMA2_Channel3);   //��DMA��ͨ��5�Ĵ�������Ϊȱʡֵ  ����1��Ӧ����DMAͨ��5
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;  //DMA����ADC����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)g_stUart4Msg.buf;  //DMA�ڴ����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //���ݴ��䷽�򣬴������ȡ���͵��ڴ�
    DMA_InitStructure.DMA_BufferSize = UART4_REC_LEN;  //DMAͨ����DMA����Ĵ�С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
    DMA_Init(DMA2_Channel3, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��
    DMA_ClearFlag(DMA2_FLAG_GL3);
    DMA_Cmd(DMA2_Channel3, ENABLE);  //��ʽ����DMA����  
}

void UART4_Init()
{
    UART4_Configuration();
}


void UART4_Send_Bytes(u8 *Data, u8 len) //���ͣ�
{
    int i;

    for(i = 0; i < len; i++)
    {
        USART_SendData(UART4,*Data);
        while( USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET );
    }
}

void UART4DmaClr(void)
{
    DMA_Cmd(DMA2_Channel3, DISABLE);
    DMA_SetCurrDataCounter(DMA2_Channel3, UART4_REC_LEN);
    DMA_ClearFlag(DMA2_FLAG_GL3);
    DMA_Cmd(DMA2_Channel3, ENABLE);
}

//�������0d 0a
void UART4_IRQHandler(void) //�жϴ�������
{    
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if(USART_GetITStatus(UART4, USART_IT_IDLE) == SET) //�ж��Ƿ����жϣ�
    {
        USART_ReceiveData(UART4);
        g_stUart4Msg.len = UART4_REC_LEN - DMA_GetCurrDataCounter(DMA2_Channel3); //����ӱ�֡���ݳ���

        //�ͷ����ݽ�������źţ������ж����洦�����ݽ���
        MessageSendFromISR(MSG_ID_WHEEL_STATE, (uint32_t)&g_stUart4Msg, &xHigherPriorityTaskWoken);

        USART_ClearITPendingBit(UART4, USART_IT_IDLE);         //����жϱ�־
        UART4DmaClr();                   //�ָ�DMAָ�룬�ȴ���һ�εĽ���
    }  
} 


