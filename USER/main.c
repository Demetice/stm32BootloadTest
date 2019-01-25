#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "log.h"
#include "msg.h"

//任务函数
void vl53l0x0_task(void *pvParameters);
//void TIM3_Int_Init(u16 arr,u16 psc);

u32 g_cnt = 0;

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	 
    delay_init();	    				//延时函数初始化	  
    uart_init(115200);					//初始化串口
    LED_Init();		  					//初始化LED
    //TIM3_Int_Init(4999,7199);//10Khz的计数频率，计数到5000为500ms  

    vl53l0x0_task(NULL);
}


//创建VL53L0X_0任务
void vl53l0x0_task(void *pvParameters)
{    
    
    while(1)
    {
        LED0=~LED0;
        LOGD("hello world 0");
        delay_ms(1000);
    }
}

//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
//因此，每次进入中断服务程序间隔时间为((1+TIM_Prescaler )/72M)*(1+TIM_Period )=((1+7199)/72M)*(1+9999)=1秒*/
void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

    //定时器TIM3初始化
    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值   
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位

    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

    //中断优先级NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

    TIM_Cmd(TIM3, ENABLE);  //使能TIMx                     
}
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 
        //LED1=!LED1;
        g_cnt++;
        if (g_cnt > 2147549183)
        {
            g_cnt = 0;
        }
    }
}
   




