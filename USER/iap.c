#include "iap.h"
#include "log.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "sys.h"
#include "delay.h"


IAP_FLASH_BUF_S g_stIapBuf;
IAP_STATE_E g_eIapState = E_IAP_STATE_NONE;

void IAP_Init(void)
{
    memset(&g_stIapBuf, 0, sizeof(g_stIapBuf));
    g_eIapState = E_IAP_STATE_NONE;
    g_stIapBuf.addr = OTA_START_ADDR;
}


int IAP_WriteFlashBuf(u8 *buff, u32 len)
{
	if (len + g_stIapBuf.bufLen[g_stIapBuf.idx] < FLASH_PAGE_SIZE)
	{
		memcpy(&g_stIapBuf.buff[g_stIapBuf.idx * FLASH_PAGE_SIZE + g_stIapBuf.bufLen[g_stIapBuf.idx]], buff, len);
		g_stIapBuf.bufLen[g_stIapBuf.idx] += len;
        printf("c\r\n");
	}
	else if(len + g_stIapBuf.bufLen[g_stIapBuf.idx] > FLASH_PAGE_SIZE)
	{
		int copySize = FLASH_PAGE_SIZE - g_stIapBuf.bufLen[g_stIapBuf.idx];
		memcpy(&g_stIapBuf.buff[g_stIapBuf.idx * FLASH_PAGE_SIZE + g_stIapBuf.bufLen[g_stIapBuf.idx]], buff, copySize);
		g_stIapBuf.bufLen[g_stIapBuf.idx] += FLASH_PAGE_SIZE;
		g_stIapBuf.complete[g_stIapBuf.idx] = 1;

		g_stIapBuf.idx++;
		if (g_stIapBuf.idx > 1)
		{
			g_stIapBuf.idx = 0;
		}

		if (g_stIapBuf.complete[g_stIapBuf.idx] == 1)
		{
			LOGD("error recevie data too fast in buff :%d", g_stIapBuf.idx);
		}

		int leftSize = len + g_stIapBuf.bufLen[g_stIapBuf.idx] - FLASH_PAGE_SIZE;
		memcpy(&g_stIapBuf.buff[g_stIapBuf.idx * FLASH_PAGE_SIZE], buff + copySize, leftSize);
		g_stIapBuf.bufLen[g_stIapBuf.idx] += leftSize;
	}
	else
	{
		memcpy(&g_stIapBuf.buff[g_stIapBuf.idx * FLASH_PAGE_SIZE + g_stIapBuf.bufLen[g_stIapBuf.idx]], buff, len);
		g_stIapBuf.bufLen[g_stIapBuf.idx] += len;
		g_stIapBuf.complete[g_stIapBuf.idx] = 1;

		g_stIapBuf.idx++;
		if (g_stIapBuf.idx > 1)
		{
			g_stIapBuf.idx = 0;
		}
	}

	return 0;
}



void IAP_SetState(IAP_STATE_E state)
{
    if (state >= E_IAP_STATE_BUTT)
    {
        LOGD("Error param");
        return;
    }

    g_eIapState = state;
}

inline IAP_STATE_E IAP_GetState(void)
{
    return g_eIapState;
}



void FLASH_WriteByte(uint32_t addr , uint8_t *p , uint16_t Byte_Num)
{
    uint32_t HalfWord;
    Byte_Num = Byte_Num/2;
    u32 numOfPage = 0;

    if(addr<FLASH_BASE||((addr+Byte_Num)>=(FLASH_BASE+1024*FLASH_SIZE)))
    {
       return;//�Ƿ���ַ
    }

    numOfPage = (Byte_Num - 1) / FLASH_PAGE_SIZE;
    uint32_t offsetAddress = addr - FLASH_BASE;               //����ȥ��0X08000000���ʵ��ƫ�Ƶ�ַ
    uint32_t sectorPosition = offsetAddress / FLASH_PAGE_SIZE;            //����������ַ������STM32F103VET6Ϊ0~255
    uint32_t sectorStartAddress = sectorPosition * FLASH_PAGE_SIZE + FLASH_BASE;    //��Ӧ�������׵�ַ
    
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    for (int i = 0; i <= numOfPage; i++)
    {
        FLASH_ErasePage(sectorStartAddress + i * FLASH_PAGE_SIZE);//�����������
    }
    
    while(Byte_Num --)
    {
        HalfWord=*(p++);
        HalfWord|=*(p++)<<8;
        FLASH_ProgramHalfWord(addr, HalfWord);
        addr += 2;
    }
    FLASH_Lock();
}

void FLASH_ReadByte(uint32_t addr , uint8_t *p , uint16_t Byte_Num)
{
    while(Byte_Num--)
    {
        *(p++)=*((uint8_t*)addr++);
    }
}

void IAP_WriteOtaData(u32 addr, u8 *data, u32 size)
{
    if ((addr < OTA_START_ADDR) || (addr + size >= OTA_END_ADDR))
    {
        return;
    }

    FLASH_WriteByte(addr, data, size);
}

void IAP_DownloadFlash()
{
    for (int i = 0; i < 2; ++i)
    {
        if (g_stIapBuf.complete[i] == 1)
        {
            IAP_WriteOtaData(g_stIapBuf.addr, &g_stIapBuf.buff[i * FLASH_PAGE_SIZE], FLASH_PAGE_SIZE);

            g_stIapBuf.complete[i] = 0;
            g_stIapBuf.bufLen[i] = 0;
            memset(&g_stIapBuf.buff[i * FLASH_PAGE_SIZE], 0, FLASH_PAGE_SIZE);
            g_stIapBuf.addr += FLASH_PAGE_SIZE;
            printf("c\r\n");
        }
    }
}

void IAP_DownloadLastPkgToFlash()
{
    for (int i = 0; i < 2; i++)
    {
        if (g_stIapBuf.bufLen[i] != 0)
        {
            g_stIapBuf.complete[i] = 1;
        }
    }

    IAP_DownloadFlash();
}

void IAP_LoadApp(u32 appxaddr)
{
    P_FUNC_JUMP jump2app;

    if(((*(vu32*)appxaddr)&0x2FFE0000)==0x20000000)	//���ջ����ַ�Ƿ�Ϸ�.
    { 
        jump2app=(void(*)())*(vu32*)(appxaddr+4);   //�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)		
        MSR_MSP(*(vu32*)appxaddr);                  //��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
        for(int i = 0; i < 8; i++)
        {
            NVIC->ICER[i] = 0xFFFFFFFF;	/* �ر��ж�*/
            NVIC->ICPR[i] = 0xFFFFFFFF;	/* ����жϱ�־λ */
        }
        jump2app();                     //��ת��APP.
    }
    else
    {
        printf("no user program\r\n");
    }
}

void IAP_ResetMCU(void)
{
    LOGD("Reset by softwore.");
    delay_ms(200);
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

void IAP_SetIapFlag(uint16_t flag)
{
    FLASH_WriteByte(FLAG_START_ADDR, (u8 *)&flag, sizeof(u16));
}

u16 IAP_ReadIapFlag(void)
{
    return *((u16*)FLAG_START_ADDR);
}

