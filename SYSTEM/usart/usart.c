#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSʹ��	 	  
#endif 

#include "msg.h"
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

 
#if EN_USART1_RX   //���ʹ���˽���
UART_MSG_S g_stUart1Msg = {0};


void uart_init(u32 bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //ʹ��DMA����
    
    //USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9

    //USART1_RX	  GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7 ;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    //USART ��ʼ������

    USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//�������ڽ����ж�
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

    //Rx DMA CONFIG 
    DMA_Cmd(DMA1_Channel5, DISABLE);   //                       
    DMA_DeInit(DMA1_Channel5);  
    
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)g_stUart1Msg.buf;        
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                    
    DMA_InitStructure.DMA_BufferSize = USART_REC_LEN;                     
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;       
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;               
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                           

    DMA_Init(DMA1_Channel5, &DMA_InitStructure);            
    DMA_ClearFlag(DMA1_FLAG_GL5);                              
    DMA_Cmd(DMA1_Channel5, ENABLE);  
}

void USART1_Send_Byte(u8 Data) //����һ���ֽڣ�
{
    USART_SendData(USART1,Data);
    while( USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET );
}

void USART1_Send_Bytes(u8 *Data, u8 len) //�����ַ�����
{
    int i;

    for(i = 0; i < len; i++)
    {
        USART1_Send_Byte(Data[i]);
    }
}


void USART1DmaClr(void)
{
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel5, USART_REC_LEN);
    DMA_ClearFlag(DMA1_FLAG_GL5);
    DMA_Cmd(DMA1_Channel5, ENABLE);
}

void USART1_IRQHandler(void) //�жϴ�������
{    
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if(USART_GetITStatus(USART1, USART_IT_IDLE) == SET) //�ж��Ƿ����жϣ�
    {
        USART_ReceiveData(USART1);
        g_stUart1Msg.len = USART_REC_LEN - DMA_GetCurrDataCounter(DMA1_Channel5); //����ӱ�֡���ݳ���
        //printf("usart1 receive data by dma len:%u\r\n", g_ucUsart1ReceiveDataLen);
        //USART1_Send_Bytes(USART_RX_BUF, g_ucUsart1ReceiveDataLen);
//        MessageSendFromISR(MSG_ID_USART1_DMA_RECEIVE, (uint32_t)&g_stUart1Msg, &xHigherPriorityTaskWoken);

        USART_ClearITPendingBit(USART1, USART_IT_IDLE);         //����жϱ�־
        USART1DmaClr();                   //�ָ�DMAָ�룬�ȴ���һ�εĽ���
    }  
} 

#endif

