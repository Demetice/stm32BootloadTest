#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
#ifdef DEBUG_ON_BOARD
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //??PB,PE????

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PB.5 ????
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //????
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO????50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 //?????????GPIOB.5
    GPIO_SetBits(GPIOB,GPIO_Pin_5);						 //PB.5 ???

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5 ????, ????
    GPIO_Init(GPIOE, &GPIO_InitStructure);	  				 //???? ,IO????50MHz
    GPIO_SetBits(GPIOE,GPIO_Pin_5); 						 //PE.5 ??? 
#else
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);	 //??PC????

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				 //LED0-->PB.5 ????
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //????
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO????50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);					 //?????????GPIOB.5
    GPIO_SetBits(GPIOC,GPIO_Pin_2);						 //PB.5 ???

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	    		 //LED1-->PE.5 ????, ????
    GPIO_Init(GPIOC, &GPIO_InitStructure);	  				 //???? ,IO????50MHz
    GPIO_SetBits(GPIOC,GPIO_Pin_3); 						 //PE.5 ???  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;               //LED1-->PE.5 ????, ????
    GPIO_Init(GPIOC, &GPIO_InitStructure);                  //???? ,IO????50MHz
    GPIO_SetBits(GPIOC,GPIO_Pin_5);                         //PE.5 ??? 
#endif

}
 
