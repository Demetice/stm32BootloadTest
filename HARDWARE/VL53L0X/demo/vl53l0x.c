#include "vl53l0x.h"
#include "log.h"
#include "stm32f10x_exti.h"

VL53L0X_Dev_t VL53L0XDevs[VL53L0X_DEVS_NUM];//�豸I2C���ݲ���

VL53L0X_RangingMeasurementData_t vl53l0x_data;//�������ṹ��

vu16 Distance_data=0;//����������

u8 g_aucAlarmFlag[VL53L0X_DEVS_NUM] = 0;

//VL53L0X������ģʽ����
//0��Ĭ��;1:�߾���;2:������;3:����
mode_data Mode_data[]=
{
    {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	 33000,
	 14,
	 10},//Ĭ��
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	 200000, 
	 14,
	 10},//�߾���
		
    {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	 33000,
	 18,
	 14},//������
	
    {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	 20000,
	 14,
	 10},//����
		
};

static void VL53L0X_GPIO_INIT(VOID)
{
    // Setup rangefinder device structs
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;             //�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //�����趨������ʼ��GPIOA

    VL53L0X_XSHUT_0 = 0;//ʧ��VL53L0X

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;             //�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //�����趨������ʼ��GPIOA

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

//API������Ϣ��ӡ
//Status�����鿴VL53L0X_Error�����Ķ���
void print_pal_error(VL53L0X_Error Status)
{
	
	char buf[VL53L0X_MAX_STRING_LENGTH];
	
	VL53L0X_GetPalErrorString(Status,buf);//����Status״̬��ȡ������Ϣ�ַ���
	
    LOGD("API Status: %i : %s\r\n",Status, buf);//��ӡ״̬�ʹ�����Ϣ
}

//����VL53L0X�豸I2C��ַ
//dev:�豸I2C�����ṹ��
//newaddr:�豸��I2C��ַ
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 sta=0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress==dev->I2cDevAddr)//���豸I2C��ַ��ɵ�ַһ��,ֱ���˳�
		return VL53L0X_ERROR_NONE;
	//�ڽ��е�һ���Ĵ�������֮ǰ����I2C��׼ģʽ(400Khz)
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
	    LOGD("Error in wrByte dev addr:%d", dev->I2cDevAddr);
		sta=0x01;//����I2C��׼ģʽ����
		goto set_error;
	}
	//����ʹ��Ĭ�ϵ�0x52��ַ��ȡһ���Ĵ���
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x02;//��ȡ�Ĵ�������
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//�����豸�µ�I2C��ַ
		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x03;//����I2C��ַ����
			goto set_error;
		}
		//�޸Ĳ����ṹ���I2C��ַ
		dev->I2cDevAddr = FinalAddress;
		//����µ�I2C��ַ��д�Ƿ�����
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x04;//��I2C��ַ��д����
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//��ӡ������?
		printf("add no set");
	}
	if(sta!=0)
	  printf("sta:0x%x\r\n",sta);
	return Status;
}

//vl53l0x��λ����
//dev:�豸I2C�����ṹ��
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;//�����豸ԭI2C��ַ

    VL53L0X_XSHUT_0 = 0;//ʧ��VL53L0X
	delay_ms(30);
    
	VL53L0X_XSHUT_0 = 1;//ʹ��VL53L0X,�ô��������ڹ���(I2C��ַ��ָ�Ĭ��0X52)
	delay_ms(30);	

    dev->I2cDevAddr = VL53L0X_DEFAULT_ADDR;
	vl53l0x_Addr_set(dev,addr);//����VL53L0X������ԭ���ϵ�ǰԭI2C��ַ
	VL53L0X_DataInit(dev);	
}

//��ʼ��vl53l0x
//dev:�豸I2C�����ṹ��
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *pMyDevice, u8 addr)
{
    LOGD("5310 init start");
    u8 mode = DEFAULT_MODE;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    pMyDevice->I2cDevAddr = VL53L0X_DEFAULT_ADDR;//I2C��ַ(�ϵ�Ĭ��0x52)
    pMyDevice->comms_type = 1;           //I2Cͨ��ģʽ
    pMyDevice->comms_speed_khz = 400;    //I2Cͨ������


    Status = vl53l0x_Addr_set(pMyDevice, addr);//����VL53L0X������I2C��ַ
    if(Status!=VL53L0X_ERROR_NONE) 
    {
        LOGD("Set address error %d", Status);
        goto error;
    }

    Status = VL53L0X_DataInit(pMyDevice);//�豸��ʼ��
    if(Status!=VL53L0X_ERROR_NONE) 
    {
        LOGD("datainit error %d", Status);
        goto error;
    }
    delay_ms(10);

    Status =vl53l0x_set_mode(pMyDevice,mode);//���þ���ģʽ
    if(Status!=VL53L0X_ERROR_NONE) 
    {
        LOGD("Set mode error %d", Status);
        goto error;
    }

    error:
    if(Status!=VL53L0X_ERROR_NONE)
    {
        LOGD("Error status");
    	print_pal_error(Status);//��ӡ������Ϣ
    	return Status;
    } 	
    return Status;
}

//VL53L0X ����ģʽ����
//dev:�豸I2C�����ṹ��
//mode: 0:Ĭ��;1:�߾���;2:������;3:����
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode)
{
	
	 VL53L0X_Error status = VL53L0X_ERROR_NONE;
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	
	 //vl53l0x_reset(dev);//��λvl53l0x(Ƶ���л�����ģʽ���׵��²ɼ��������ݲ�׼���������һ����)
	 status = VL53L0X_StaticInit(dev);

	 status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref�ο�У׼
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);//ִ�вο�SPAD����
	 if(status!=VL53L0X_ERROR_NONE) goto error;
     delay_ms(2);		 	 
	 status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_SINGLE_RANGING);//ʹ�ܵ��β���ģʽ
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,1);//ʹ��SIGMA��Χ���
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);//ʹ���ź����ʷ�Χ���
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,Mode_data[mode].sigmaLimit);//�趨SIGMA��Χ
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,Mode_data[mode].signalLimit);//�趨�ź����ʷ�Χ��Χ
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,Mode_data[mode].timingBudget);//�趨��������ʱ��
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);//�趨VCSEL��������
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);//�趨VCSEL�������ڷ�Χ
	 
	 error://������Ϣ
	 if(status!=VL53L0X_ERROR_NONE)
	 {
		print_pal_error(status);
		return status;
	 }
	 return status;
}	

//VL53L0X ���ξ����������
//dev:�豸I2C�����ṹ��
//pdata:����������ݽṹ��
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);//ִ�е��β�ಢ��ȡ����������
	if(status !=VL53L0X_ERROR_NONE)
    {
        LOGD("Single test errror");
        return status;
    }
	Distance_data = pdata->RangeMilliMeter;//�������һ�β���������
	
  return status;
}

/**
  *@brief   NVIC_Configurationʵ��NVIC����
  *
  */
static void NVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);           //����NVIC���ȼ�����Ϊ1

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;        //�ж�Դ��[9:5]��λ�ڡ�stm32f10x.h����
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ���1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //�����ȼ���1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

//�ж����ó�ʼ��
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

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5); //ѡ��EXTI�ź�Դ
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11); //ѡ��EXTI�ź�Դ
    
    EXTI_InitTypeDef EXTI_InitStructure;                                                                                            
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;               //�ж���ѡ��
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;      //EXTIΪ�ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //�½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                //ʹ���ж�
    EXTI_Init(&EXTI_InitStructure); 

    EXTI_InitStructure.EXTI_Line = EXTI_Line11;               //�ж���ѡ��
    EXTI_Init(&EXTI_InitStructure); 

    NVIC_Configuration();
}

//�ⲿ�жϷ�����
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5)!= RESET)  
    { 
        g_aucAlarmFlag[0]=1;//��־
        //���LINE5�ϵ��жϱ�־λ 
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}

//�ⲿ�жϷ�����
void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line11)!= RESET)  
    { 
        g_aucAlarmFlag[1]=1;//��־
        //���LINE5�ϵ��жϱ�־λ 
        EXTI_ClearITPendingBit(EXTI_Line11);
    }
}


void VL53L0X_SET_CONTINUOUS_MODE(VL53L0X_Dev_t *dev, u8 mode)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;//����״̬

    status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);//ʹ����������ģʽ
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev,Mode_data[mode].timingBudget);//�����ڲ����ڲ���ʱ��
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,1);//ʹ��SIGMA��Χ���
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);//ʹ���ź����ʷ�Χ���
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,Mode_data[mode].sigmaLimit);//�趨SIGMA��Χ
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2); 
    status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,Mode_data[mode].signalLimit);//�趨�ź����ʷ�Χ��Χ
    if(status!=VL53L0X_ERROR_NONE) goto error; 
    delay_ms(2);
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,Mode_data[mode].timingBudget);//�趨��������ʱ��
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2); 
    status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);//�趨VCSEL��������
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    //�趨VCSEL�������ڷ�Χ
    status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_StopMeasurement(dev);//ֹͣ����
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetInterruptThresholds(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, 60<<16, 150<<16);//�趨�����ж��ϡ�����ֵ
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_SetGpioConfig(dev,0,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,
    VL53L0X_INTERRUPTPOLARITY_LOW);//�趨�����ж�ģʽ �½���
    if(status!=VL53L0X_ERROR_NONE) goto error;
    delay_ms(2);
    status = VL53L0X_ClearInterruptMask(dev,0);//���VL53L0X�жϱ�־λ

    error://������Ϣ
    if(status!=VL53L0X_ERROR_NONE)
    {
    print_pal_error(status);
    return ;
    }

    VL53L0X_StartMeasurement(dev);//��������

    return;
}


//vl53l0x�жϲ���ģʽ����
//dev:�豸I2C�����ṹ��
//mode: 0:Ĭ��;1:�߾���;2:������;3:����
void vl53l0x_interrupt_start(VL53L0X_Dev_t *dev)
{

    exti_init();//�жϳ�ʼ��

    for (int i = 0; i < VL53L0X_DEVS_NUM; ++i)
        VL53L0X_SET_CONTINUOUS_MODE(&dev[i], DEFAULT_MODE);

    memset(g_aucAlarmFlag, 0, sizeof(g_aucAlarmFlag));    

    while(1)
    {  
        TickType_t cur_time = xTaskGetTickCount();
        const portTickType xFrequency = pdMS_TO_TICKS(20); //�������� 1/20 = 50ms

        for (int i = 0; i < VL53L0X_DEVS_NUM; ++i)
        {
            if(g_aucAlarmFlag[i] == 1)//�����ж�
            {
                g_aucAlarmFlag[i] = 0;
                VL53L0X_GetRangingMeasurementData(&dev[i],&vl53l0x_data);//��ȡ��������,������ʾ����
                LOGD("dev %d : %3d mm", i, vl53l0x_data.RangeMilliMeter);
                vTaskDelay(1);
                VL53L0X_ClearInterruptMask(&dev[i],0);//���VL53L0X�жϱ�־λ 
            }        
        }

        vTaskDelayUntil(&cur_time, xFrequency);
    }
}




//������ͨ����
//dev���豸I2C�����ṹ��
//modeģʽ���� 0:Ĭ��;1:�߾���;2:������
void vl53l0x_general_start(void)
{
    vl53l0x_interrupt_start(VL53L0XDevs);
}
