#ifndef __WHEEL_DRV_ZLAC706_H__
#define __WHEEL_DRV_ZLAC706_H__

#include "uart4.h"

typedef enum 
{
    E_ZLAC_CMD_START = 0, //�������
    E_ZLAC_CMD_STOP = 1, //���ֹͣ
    E_ZLAC_CMD_PC_MODE = 2, //ѡ��Ϊuart ��ֵģʽ
    E_ZLAC_CMD_PID_P = 3, //pid P����
    E_ZLAC_CMD_PID_I = 4, //PID I ����
    E_ZLAC_CMD_PID_D = 5, //PID D ����
    E_ZLAC_CMD_SPEED_INCREASE = 6, //���ٶ����ã���λΪ���ٶȣ���λΪ���ٶ�
    E_ZLAC_CMD_SET_SPEED = 7, //�����ٶ�
    E_ZLAC_CMD_FIND_Z_ORIGIN = 8, //����Z��еԭ��
    E_ZLAC_CMD_CLEAR_ERROR = 9, //�������
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

