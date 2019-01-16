#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "vl53l0x.h"
#include "log.h"


//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		128  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define LED0_TASK_PRIO		2
//�����ջ��С	
#define LED0_STK_SIZE 		50  
//������
TaskHandle_t LED0Task_Handler;
//������
void led0_task(void *pvParameters);

//�������ȼ�
#define VL53L0X0_TASK_PRIO		3
//�����ջ��С	
#define VL53L0X0_STK_SIZE 		1024
//������
TaskHandle_t VL53L0X0Task_Handler;
//������
void vl53l0x0_task(void *pvParameters);

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
    delay_init();	    				//��ʱ������ʼ��	  
    uart_init(115200);					//��ʼ������
    LED_Init();		  					//��ʼ��LED
    UART4_Configuration();

     
    //������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
}

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    //����LED0����
    xTaskCreate((TaskFunction_t )led0_task,     	
                (const char*    )"led0_task",   	
                (uint16_t       )LED0_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED0_TASK_PRIO,	
                (TaskHandle_t*  )&LED0Task_Handler);   
    //����VL53L0X_0����
    xTaskCreate((TaskFunction_t )vl53l0x0_task,     
                (const char*    )"vl53l0x_0_task",   
                (uint16_t       )VL53L0X0_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )VL53L0X0_TASK_PRIO,
                (TaskHandle_t*  )&VL53L0X0Task_Handler);         
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

//LED0������ 
void led0_task(void *pvParameters)
{
    while(1)
    {
        LED1=~LED1;
        LOGD("hello world");
        vTaskDelay(5000);
    }
}   

//����VL53L0X_0����
void vl53l0x0_task(void *pvParameters)
{
    while(1)
    {
        LED0=~LED0;
        LOGD("hello world 0");
        vTaskDelay(2000);
    }

    //VL53L0X_i2c_init();//��ʼ��VL53L0X��IIC
    //VL53L0X_begin();//��ʼ��ÿ��VL53L0X�豸
    //vl53l0x_general_start();
}
