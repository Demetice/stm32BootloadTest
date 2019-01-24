#ifndef __WHEEL_DRV_ZLAC706_H__
#define __WHEEL_DRV_ZLAC706_H__

#include "uart4.h"

typedef enum 
{
    E_ZLAC_CMD_START = 0, //电机启动
    E_ZLAC_CMD_STOP = 1, //电机停止
    E_ZLAC_CMD_PC_MODE = 2, //选择为uart 数值模式
    E_ZLAC_CMD_PID_P = 3, //pid P设置
    E_ZLAC_CMD_PID_I = 4, //PID I 设置
    E_ZLAC_CMD_PID_D = 5, //PID D 设置
    E_ZLAC_CMD_SPEED_INCREASE = 6, //加速度设置，高位为加速度，低位为减速度
    E_ZLAC_CMD_SET_SPEED = 7, //设置速度
    E_ZLAC_CMD_FIND_Z_ORIGIN = 8, //设置Z机械原点
    E_ZLAC_CMD_CLEAR_ERROR = 9, //清除故障
}ZLAC_CMD_E;

typedef struct tagZlacCmd
{
    u8 cmd;
    u8 high;
    u8 low;
    u8 chksum;
}ZLAC_CMD_S;

extern int WHEEL_SendCmdRight(ZLAC_CMD_S *pstCmd);


#endif

