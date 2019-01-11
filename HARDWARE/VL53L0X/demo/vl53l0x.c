#include "vl53l0x.h"
#include "log.h"
#include "stm32f10x_exti.h"

VL53L0X_Dev_t VL53L0XDevs[VL53L0X_DEVS_NUM];//设备I2C数据参数

VL53L0X_RangingMeasurementData_t vl53l0x_data;//测距测量结构体

vu16 Distance_data=0;//保存测距数据

u8 g_aucAlarmFlag[VL53L0X_DEVS_NUM] = 0;

//VL53L0X各精度模式参数
//0：默认;1:高精度;2:长距离;3:高速
mode_data Mode_data[]=
{
    {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	 33000,
	 14,
	 10},//默认
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	 200000, 
	 14,
	 10},//高精度
		
    {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	 33000,
	 18,
	 14},//长距离
	
    {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	 20000,
	 14,
	 10},//高速
		
};

static void VL53L0X_GPIO_INIT(VOID)
{
    // Setup rangefinder device structs
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;             //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO口速度为50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //根据设定参数初始化GPIOA

    VL53L0X_XSHUT_0 = 0;//失能VL53L0X

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;             //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO口速度为50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //根据设定参数初始化GPIOA

    VL53L0X_XSHUT_1 = 0;

    return;
}

//test
void VL53L0X_begin(void)
{
    int cnt = 0;
    VL53L0X_Error rtn;

    VL53L0X_GPIO_INIT();

    vTaskDelay(30);

    for (cnt = 0; cnt < VL53L0X_DEVS_NUM; ++cnt)
    {
        while(1)
        {
            if (cnt == 0)
            {
                VL53L0X_XSHUT_0 = 1; 
            }
            else
            {
                VL53L0X_XSHUT_1 = 1;
            }
            
            vTaskDelay(50);            
            
            if (VL53L0X_ERROR_NONE  == vl53l0x_init(&VL53L0XDevs[cnt], VL53L0X_DEFAULT_ADDR + (cnt + 1) * 2))
            {
                LOGD("Init device %d success", cnt);
                break;
            }

            LOGD("Init vl5310x dev %d error", cnt);
            vTaskDelay(1000);
        }
        vTaskDelay(1000);        
    }
    
    LOGD("VL53L0X begin init finish");
    return ;
}

//API错误信息打印
//Status：详情看VL53L0X_Error参数的定义
void print_pal_error(VL53L0X_Error Status)
{
	
	char buf[VL53L0X_MAX_STRING_LENGTH];
	
	VL53L0X_GetPalErrorString(Status,buf);//根据Status状态获取错误信息字符串
	
    LOGD("API Status: %i : %s\r\n",Status, buf);//打印状态和错误信息
}

//配置VL53L0X设备I2C地址
//dev:设备I2C参数结构体
//newaddr:设备新I2C地址
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 sta=0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress==dev->I2cDevAddr)//新设备I2C地址与旧地址一致,直接退出
		return VL53L0X_ERROR_NONE;
	//在进行第一个寄存器访问之前设置I2C标准模式(400Khz)
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
	    LOGD("Error in wrByte dev addr:%d", dev->I2cDevAddr);
		sta=0x01;//设置I2C标准模式出错
		goto set_error;
	}
	//尝试使用默认的0x52地址读取一个寄存器
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x02;//读取寄存器出错
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//设置设备新的I2C地址
		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x03;//设置I2C地址出错
			goto set_error;
		}
		//修改参数结构体的I2C地址
		dev->I2cDevAddr = FinalAddress;
		//检查新的I2C地址读写是否正常
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x04;//新I2C地址读写出错
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//打印错误信?
		printf("add no set");
	}
	if(sta!=0)
	  printf("sta:0x%x\r\n",sta);
	return Status;
}

//vl53l0x复位函数
//dev:设备I2C参数结构体
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;//保存设备原I2C地址

    VL53L0X_XSHUT_0 = 0;//失能VL53L0X
	delay_ms(30);
    
	VL53L0X_XSHUT_0 = 1;//使能VL53L0X,让传感器处于工作(I2C地址会恢复默认0X52)
	delay_ms(30);	

    dev->I2cDevAddr = VL53L0X_DEFAULT_ADDR;
	vl53l0x_Addr_set(dev,addr);//设置VL53L0X传感器原来上电前原I2C地址
	VL53L0X_DataInit(dev);	
}

//初始化vl53l0x
//dev:设备I2C参数结构体
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *pMyDevice, u8 addr)
{
    LOGD("5310 init start");
    u8 mode = DEFAULT_MODE;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    pMyDevice->I2cDevAddr = VL53L0X_DEFAULT_ADDR;//I2C地址(上电默认0x52)
    pMyDevice->comms_type = 1;           //I2C通信模式
    pMyDevice->comms_speed_khz = 400;    //I2C通信速率


    Status = vl53l0x_Addr_set(pMyDevice, addr);//设置VL53L0X传感器I2C地址
    if(Status!=VL53L0X_ERROR_NONE) 
    {
        LOGD("Set address error %d", Status);
        goto error;
    }

    Status = VL53L0X_DataInit(pMyDevice);//设备初始化
    if(Status!=VL53L0X_ERROR_NONE) 
    {
        LOGD("datainit error %d", Status);
        goto error;
    }
    delay_ms(10);

    Status =vl53l0x_set_mode(pMyDevice,mode);//配置精度模式
    if(Status!=VL53L0X_ERROR_NONE) 
    {
        LOGD("Set mode error %d", Status);
        goto error;
    }

    error:
    if(Status!=VL53L0X_ERROR_NONE)
    {
        LOGD("Error status");
    	print_pal_error(Status);//打印错误信息
    	return Status;
    } 	
    return Status;
}

//VL53L0X 测量模式配置
//dev:设备I2C参数结构体
//mode: 0:默认;1:高精度;2:长距离;3:高速
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode)
{
	
	 VL53L0X_Error status = VL53L0X_ERROR_NONE;
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	
	 //vl53l0x_reset(dev);//复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)
	 status = VL53L0X_StaticInit(dev);

	 status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref参考校准
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);//执行参考SPAD管理
	 if(status!=VL53L0X_ERROR_NONE) goto error;
     delay_ms(2);		 	 
	 status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_SINGLE_RANGING);//使能单次测量模式
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,1);//使能SIGMA范围检查
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);//使能信号速率范围检查
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,Mode_data[mode].sigmaLimit);//设定SIGMA范围
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,Mode_data[mode].signalLimit);//设定信号速率范围范围
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,Mode_data[mode].timingBudget);//设定完整测距最长时间
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);//设定VCSEL脉冲周期
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);//设定VCSEL脉冲周期范围
	 
	 error://错误信息
	 if(status!=VL53L0X_ERROR_NONE)
	 {
		print_pal_error(status);
		return status;
	 }
	 return status;
}	

//VL53L0X 单次距离测量函数
//dev:设备I2C参数结构体
//pdata:保存测量数据结构体
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);//执行单次测距并获取测距测量数据
	if(status !=VL53L0X_ERROR_NONE)
    {
        LOGD("Single test errror");
        return status;
    }
	Distance_data = pdata->RangeMilliMeter;//保存最近一次测距测量数据
	
  return status;
}

/**
  *@brief   NVIC_Configuration实现NVIC配置
  *
  */
static void NVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);           //配置NVIC优先级分组为1

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;        //中断源：[9:5]，位于“stm32f10x.h”中
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级：1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级：1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //使能中断通道
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

//中断配置初始化
static void exti_init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;                      

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | \
                            RCC_APB2Periph_AFIO, ENABLE);     
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;                 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;             
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                             
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5); //选择EXTI信号源
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11); //选择EXTI信号源
    
    EXTI_InitTypeDef EXTI_InitStructure;                                                                                            
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;               //中断线选择
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;      //EXTI为中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                //使能中断
    EXTI_Init(&EXTI_InitStructure); 

    EXTI_InitStructure.EXTI_Line = EXTI_Line11;               //中断线选择
    EXTI_Init(&EXTI_InitStructure); 

    NVIC_Configuration();
}

//外部中断服务函数
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5)!= RESET)  
    { 
        g_aucAlarmFlag[0]=1;//标志
        //清除LINE5上的中断标志位 
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}

//外部中断服务函数
void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line11)!= RESET)  
    { 
        g_aucAlarmFlag[1]=1;//标志
        //清除LINE5上的中断标志位 
        EXTI_ClearITPendingBit(EXTI_Line11);
    }
}


void VL53L0X_SET_CONTINUOUS_MODE(VL53L0X_Dev_t *dev, u8 mode)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;//工作状态

    status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);//使能连续测量模式
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev,Mode_data[mode].timingBudget);//设置内部周期测量时间
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,1);//使能SIGMA范围检查
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);//使能信号速率范围检查
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,Mode_data[mode].sigmaLimit);//设定SIGMA范围
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2); 
    status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,Mode_data[mode].signalLimit);//设定信号速率范围范围
    if(status!=VL53L0X_ERROR_NONE) goto error; 
    delay_ms(2);
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,Mode_data[mode].timingBudget);//设定完整测距最长时间
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2); 
    status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);//设定VCSEL脉冲周期
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    //设定VCSEL脉冲周期范围
    status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_StopMeasurement(dev);//停止测量
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetInterruptThresholds(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, 60<<16, 150<<16);//设定触发中断上、下限值
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetGpioConfig(dev,0,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,
    VL53L0X_INTERRUPTPOLARITY_LOW);//设定触发中断模式 下降沿
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_ClearInterruptMask(dev,0);//清除VL53L0X中断标志位

    error://错误信息
    if(status!=VL53L0X_ERROR_NONE)
    {
    print_pal_error(status);
    return ;
    }

    VL53L0X_StartMeasurement(dev);//启动测量

    return;
}


//vl53l0x中断测量模式测试
//dev:设备I2C参数结构体
//mode: 0:默认;1:高精度;2:长距离;3:高速
void vl53l0x_interrupt_start(VL53L0X_Dev_t *dev)
{

    exti_init();//中断初始化

    for (int i = 0; i < VL53L0X_DEVS_NUM; ++i)
        VL53L0X_SET_CONTINUOUS_MODE(&dev[i], DEFAULT_MODE);

    memset(g_aucAlarmFlag, 0, sizeof(g_aucAlarmFlag));    

    while(1)
    {  
        TickType_t cur_time = xTaskGetTickCount();
        const portTickType xFrequency = pdMS_TO_TICKS(20); //采样周期 1/20 = 50ms

        for (int i = 0; i < VL53L0X_DEVS_NUM; ++i)
        {
            if(g_aucAlarmFlag[i] == 1)//触发中断
            {
                g_aucAlarmFlag[i] = 0;
                VL53L0X_GetRangingMeasurementData(&dev[i],&vl53l0x_data);//获取测量距离,并且显示距离
                LOGD("dev %d : %3d mm", i, vl53l0x_data.RangeMilliMeter);
                vTaskDelay(1);
                VL53L0X_ClearInterruptMask(&dev[i],0);//清除VL53L0X中断标志位 
            }        
        }

        vTaskDelayUntil(&cur_time, xFrequency);
    }
}




//启动普通测量
//dev：设备I2C参数结构体
//mode模式配置 0:默认;1:高精度;2:长距离
void vl53l0x_general_start(void)
{
    vl53l0x_interrupt_start(VL53L0XDevs);
}
