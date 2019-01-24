#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "log.h"
#include "msg.h"

//任务函数
void vl53l0x0_task(void *pvParameters);

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	 
    delay_init();	    				//延时函数初始化	  
    uart_init(115200);					//初始化串口
    LED_Init();		  					//初始化LED

     
    vl53l0x0_task(NULL);
}

//创建VL53L0X_0任务
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

// 从backup_addr拷贝info->size的大小到app_addr地址处
u8 FlashCopy(uint32_t app_addr, uint32_t backup_addr, iap_t *info)
{
  uint8_t upgrade_buffer[FLASH_SECTOR_SIZE];
  uint16_t pageremain =  FLASH_SECTOR_SIZE - backup_addr % FLASH_SECTOR_SIZE; // 单页剩余字节
 
  if(((app_addr + info->size - 1) > APP_END_ADDR) || (app_addr < APP_START_ADDR))
  {
    return COPY_FALSE;
  }
 
  if(info->size <= pageremain) // 程序总大小小于等于单页大小
  {
    pageremain = info->size;
  }
  FlashErase(app_addr, APP_BLOCK); 
 
  while(1)
  {
    // 分页写入
    memset(upgrade_buffer, 0, sizeof(upgrade_buffer));
    spiFlashRead(backup_addr, pageremain, upgrade_buffer); // 从备份区读出pageremain字节数
    FlashWrite(app_addr, upgrade_buffer, pageremain);      // 写到程序运行的地址处
 
    if(info->size == pageremain)
    {
      break; // 写入结束
    }
    else
    {
      backup_addr += pageremain;
      app_addr += pageremain;
      info->size -= pageremain; // 减去已经写入了的字节数，地址都往后面偏移
 
      if(info->size > FLASH_SECTOR_SIZE)
      {
        pageremain = FLASH_SECTOR_SIZE; // 超过1页数据，一页一页写入
      }
      else
      {
        pageremain = info->size; // 不够1页数据
      }
    }
  }
 
  return COPY_OK;
}


