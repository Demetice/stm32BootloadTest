#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "sys.h"
#include "delay.h"
#include "led.h"
//sensors numbers
//#define NUM_RANGEFINDERS 4

//VL53L0X�������ϵ�Ĭ��IIC��ַΪ0X52(���������λ)
#define VL53L0X_Addr 0x52

//����Xshut��ƽ,�Ӷ�ʹ��VL53L0X���� 1:ʹ�� 0:�ر�
#define VL53L0X_Xshut_0 PAout(4)	
#define VL53L0X_Xshut_1 PAout(5)	


//ʹ��2.8V IO��ƽģʽ
#define USE_I2C_2V8  1

//VL53L0X_Dev_t VL53L0XDevs[NUM_RANGEFINDERS];

//����ģʽ
#define Default_Mode   0// Ĭ��
#define HIGH_ACCURACY  1//�߾���
#define LONG_RANGE     2//������
#define HIGH_SPEED     3//����

//vl53l0xģʽ���ò�����
typedef __packed struct
{
	FixPoint1616_t signalLimit;    //Signal������ֵ 
	FixPoint1616_t sigmaLimit;     //Sigmal������ֵ
	uint32_t timingBudget;         //����ʱ������
	uint8_t preRangeVcselPeriod ;  //VCSEL��������
	uint8_t finalRangeVcselPeriod ;//VCSEL�������ڷ�Χ
	
}mode_data;


extern mode_data Mode_data[];
extern VL53L0X_RangingMeasurementData_t vl53l0x_data;


VL53L0X_Error vl53l0x_init(void);//��ʼ��vl53l0x
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr);
void VL53L0X_begin(void);
void print_pal_error(VL53L0X_Error Status);//������Ϣ��ӡ
void vl53l0x_reset(VL53L0X_Dev_t *dev);//vl53l0x��λ

VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode);
void vl53l0x_general_start(void);
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata);
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode);

void vl53l0x_info(void);//��ȡvl53l0x�豸ID��Ϣ
void One_measurement(u8 mode);//��ȡһ�β�����������
#endif


