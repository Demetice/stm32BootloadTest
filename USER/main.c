#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "log.h"
#include "msg.h"
#include "iap.h"

#define BOOTLOAD_SHOW_TIME 0x6

int main(void)
{
    int max_times = BOOTLOAD_SHOW_TIME;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
    delay_init();	    				//��ʱ������ʼ��	  
    uart_init(115200);					//��ʼ������
    LED_Init();		  					//��ʼ��LED
    IAP_Init();

    if (IAP_ReadIapFlag() != 0)
    {
        max_times = 0xffff;
    }
    
    while (1)
    {
        for (int i = 0; i < max_times; i++)
        {
            printf("Bootload start.\r\n");
            
            LED0 = ~LED0;
            LED1 = ~LED0;
            delay_ms(500);    

            if (E_IAP_STATE_DOWNLOADING == IAP_GetState())
            {
                while(E_IAP_STATE_DOWNLOAD_COMPLETE != IAP_GetState())
                {
                    IAP_DownloadFlash();//��Ҫ�������һ����flash
                }
            }

            if (E_IAP_STATE_DOWNLOAD_COMPLETE == IAP_GetState())
            {
                IAP_DownloadLastPkgToFlash();
                IAP_CopyProgramToAppArea();
                break;
            }
        }    

        IAP_SetIapFlag(0);
        LOGD("Start app...");
        IAP_LoadApp(APP_START_ADDR);
        IAP_SetState(E_IAP_STATE_NONE);
    }        
}



