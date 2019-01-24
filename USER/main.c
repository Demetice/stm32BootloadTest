#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "log.h"
#include "msg.h"

//������
void vl53l0x0_task(void *pvParameters);

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
    delay_init();	    				//��ʱ������ʼ��	  
    uart_init(115200);					//��ʼ������
    LED_Init();		  					//��ʼ��LED

     
    vl53l0x0_task(NULL);
}

//����VL53L0X_0����
void vl53l0x0_task(void *pvParameters)
{
    int led_state = 1;
    
    
    while(1)
    {
        LED0=~LED0;
        LOGD("hello world 0");
        delay_ms(1000);
        led_state = !led_state;
    }
}

// ��backup_addr����info->size�Ĵ�С��app_addr��ַ��
u8 FlashCopy(uint32_t app_addr, uint32_t backup_addr, iap_t *info)
{
  uint8_t upgrade_buffer[FLASH_SECTOR_SIZE];
  uint16_t pageremain =  FLASH_SECTOR_SIZE - backup_addr % FLASH_SECTOR_SIZE; // ��ҳʣ���ֽ�
 
  if(((app_addr + info->size - 1) > APP_END_ADDR) || (app_addr < APP_START_ADDR))
  {
    return COPY_FALSE;
  }
 
  if(info->size <= pageremain) // �����ܴ�СС�ڵ��ڵ�ҳ��С
  {
    pageremain = info->size;
  }
  FlashErase(app_addr, APP_BLOCK); 
 
  while(1)
  {
    // ��ҳд��
    memset(upgrade_buffer, 0, sizeof(upgrade_buffer));
    spiFlashRead(backup_addr, pageremain, upgrade_buffer); // �ӱ���������pageremain�ֽ���
    FlashWrite(app_addr, upgrade_buffer, pageremain);      // д���������еĵ�ַ��
 
    if(info->size == pageremain)
    {
      break; // д�����
    }
    else
    {
      backup_addr += pageremain;
      app_addr += pageremain;
      info->size -= pageremain; // ��ȥ�Ѿ�д���˵��ֽ�������ַ��������ƫ��
 
      if(info->size > FLASH_SECTOR_SIZE)
      {
        pageremain = FLASH_SECTOR_SIZE; // ����1ҳ���ݣ�һҳһҳд��
      }
      else
      {
        pageremain = info->size; // ����1ҳ����
      }
    }
  }
 
  return COPY_OK;
}


