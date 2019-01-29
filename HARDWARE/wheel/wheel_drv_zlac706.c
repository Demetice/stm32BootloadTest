#include "wheel_drv_zlac706.h"
#include "log.h"
#include "public.h"


void WHEEL_DRV_INIT()
{
    UART4_Init();
}

int WHEEL_SendCmd(ZLAC_CMD_S *pstCmd)
{
    PUB_CHECK_POINT(pstCmd);

    pstCmd->chksum = (u8)(pstCmd->cmd + pstCmd->high + pstCmd->low);

    UART4_Send_Bytes((u8*)pstCmd, sizeof(ZLAC_CMD_S));

    return E_RTN_OK;
}

int WHEEL_sendRequirstOfReadStaus(void)
{
    u8 buf[] = {0x80, 0, 0x80};

    UART4_Send_Bytes(buf, sizeof(buf));

    return E_RTN_OK;
}

void WHEEL_DRV_SET_PC_MODE()
{

}

