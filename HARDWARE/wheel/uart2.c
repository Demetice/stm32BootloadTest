#include "uart2.h"
#include "msg.h"

UART4_MSG_S g_stUart2Msg;


/*���������ӵ���PC10 PC11�ӿڣ���UART4
  ��������ǳ�ʼ����ص�USART����*/
void UART2_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //UART4 TX��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�������������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); //�˿�C��
        
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //UART4 RX��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //�������룻
    GPIO_Init(GPIOA, &GPIO_InitStructure); //�˿�C��

    USART_InitStructure.USART_BaudRate = WHEEL_ADP_USART_BAUD_RATE; //�����ʣ�
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //����λ8λ��
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //ֹͣλ1λ��
    USART_InitStructure.USART_Parity = USART_Parity_No ; //��У��λ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //��Ӳ�����أ�
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //�շ�ģʽ��
    USART_Init(USART2, &USART_InitStructure);//���ô��ڲ�����

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //�����ж��飬4λ��ռ���ȼ���4λ��Ӧ���ȼ���

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //�жϺţ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7; //��ռ���ȼ���
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //��Ӧ���ȼ���
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE); //ʹ�ܴ��ڣ�

}

void UART2_Init()
{
    UART2_Configuration();
}


void UART2_Send_Bytes(u8 *Data, u8 len) //���ͣ�
{
    int i;

    for(i = 0; i < len; i++)
    {
        USART_SendData(USART2,Data[i]);
        while( USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET );
    }
}

//�������0d 0a
void UART2_IRQHandler(void) //�жϴ�������
{    
    if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET) //�ж��Ƿ����жϣ�
    {
        USART_ReceiveData(USART2);

        USART_SendData(USART2, 96);
        while( USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET );
                
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);         //����жϱ�־
    }  
} 



