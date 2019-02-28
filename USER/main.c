#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "log.h"
#include "msg.h"
#include "iap.h"


int main(void)
{    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
    delay_init();	    				//��ʱ������ʼ��	  
    uart_init(115200);					//��ʼ������
    LED_Init();		  					//��ʼ��LED
    IAP_Init();

    if (IAP_ReadIapFlag() != 0)
    {
        IAP_SetIapFlag(0);
        LOGD("Start app...");
        IAP_LoadApp(APP_START_ADDR);
        IAP_SetState(E_IAP_STATE_NONE);
    }
    
    while (1)
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
            IAP_SetIapFlag(0);
            IAP_ResetMCU();
            break;
        } 
    }        
}



