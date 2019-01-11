#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "sys.h"
#include "delay.h"
#include "led.h"
//sensors numbers
//#define NUM_RANGEFINDERS 4

//VL53L0X传感器上电默认IIC地址为0X52(不包含最低位)
#define VL53L0X_Addr 0x52

//控制Xshut电平,从而使能VL53L0X工作 1:使能 0:关闭
#define VL53L0X_Xshut_0 PAout(4)	
#define VL53L0X_Xshut_1 PAout(5)	


//使能2.8V IO电平模式
#define USE_I2C_2V8  1

//VL53L0X_Dev_t VL53L0XDevs[NUM_RANGEFINDERS];

//测量模式
#define Default_Mode   0// 默认
#define HIGH_ACCURACY  1//高精度
#define LONG_RANGE     2//长距离
#define HIGH_SPEED     3//高速

//vl53l0x模式配置参数集
typedef __packed struct
{
	FixPoint1616_t signalLimit;    //Signal极限数值 
	FixPoint1616_t sigmaLimit;     //Sigmal极限数值
	uint32_t timingBudget;         //采样时间周期
	uint8_t preRangeVcselPeriod ;  //VCSEL脉冲周期
	uint8_t finalRangeVcselPeriod ;//VCSEL脉冲周期范围
	
}mode_data;


extern mode_data Mode_data[];
extern VL53L0X_RangingMeasurementData_t vl53l0x_data;


VL53L0X_Error vl53l0x_init(void);//初始化vl53l0x
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr);
void VL53L0X_begin(void);
void print_pal_error(VL53L0X_Error Status);//错误信息打印
void vl53l0x_reset(VL53L0X_Dev_t *dev);//vl53l0x复位

VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode);
void vl53l0x_general_start(void);
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata);
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode);

void vl53l0x_info(void);//获取vl53l0x设备ID信息
void One_measurement(u8 mode);//获取一次测量距离数据
#endif


