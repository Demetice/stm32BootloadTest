#include "stm32f10x_i2c.h"
#include "log.h"

//定义i2c 的速度，比如100kHz 100000
#define I2C_ADP_SPEED 400000

void I2C_ADP_InitGPIO(void)
{
    GPIO_InitTypeDef stGpioInit;

    stGpioInit.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    stGpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    stGpioInit.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &stGpioInit);
}

void I2C_ADP_RccConfiguration(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM4, ENABLE);
}

void I2C_ADP_Configuration(void)
{
    I2C_InitTypeDef stI2cInit;

    stI2cInit.I2C_Ack = I2C_Ack_Enable;
    stI2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    stI2cInit.I2C_ClockSpeed = I2C_ADP_SPEED;
    stI2cInit.I2C_DutyCycle = I2C_DutyCycle_2;
    stI2cInit.I2C_Mode  = I2C_Mode_I2C;
    stI2cInit.I2C_OwnAddress1 = 0xa0;
}

void I2C_ADP_Init()
{
    I2C_ADP_InitGPIO();
    I2C_ADP_RccConfiguration();
    I2C_ADP_Configuration();
}


