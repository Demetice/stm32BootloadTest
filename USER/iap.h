#ifndef __IAP_H__
#define __IAP_H__

#include "stm32f10x.h"
#include "stm32f10x_flash.h"

#define FLASH_SIZE 512

#if FLASH_SIZE<256
#define FLASH_PAGE_SIZE    ((uint16_t)1024) //һҳΪ2K��С
#else
#define FLASH_PAGE_SIZE    ((uint16_t)2048) //һҳΪ2K��С
#endif

//app ��ʼ��ַ
#define APP_START_ADDR  (uint32_t)0x08005000 
#define APP_END_ADDR    (uint32_t)0x08036FFF

//OTA �����ļ���ŵ�ַ
//#define OTA_START_ADDR   ((uint32_t) 0x08037000)//д�����ʼ��ַ
//#define OTA_END_ADDR     ((uint32_t)0x08068FFF)//������ַ

#define OTA_START_ADDR   APP_START_ADDR  //д�����ʼ��ַ
#define OTA_END_ADDR     APP_END_ADDR    //������ַ


//FLAG ��ʼ��ַ
#define FLAG_START_ADDR (uint32_t)0x08069000
#define FLAG_END_ADDR   (uint32_t)0x0807FFFF

typedef enum
{
    E_IAP_STATE_NONE = 0,
    E_IAP_STATE_DOWNLOADING = 1,
    E_IAP_STATE_DOWNLOAD_COMPLETE = 2,
    E_IAP_STATE_BUTT
}IAP_STATE_E;


//������ת����ָ������
typedef void (*P_FUNC_JUMP)(void);

//дflash �Ļ���buff, uart���ڽ���������д��buff�ڣ� buff �����Ժ���дflash
typedef struct tagIapFlashBuff
{
    u8 buff[FLASH_PAGE_SIZE * 2]; //������buff,��uartд��һ������д��һ��
    u8 idx;
    u8 rsv;
    u8 complete[2];
    u32 bufLen[2];
    u32 addr;
}IAP_FLASH_BUF_S;


typedef struct tagIapCmdHdr
{
    u8 start;
    u8 len;
    u8 cmd;
    u8 ver;
}IAP_CMD_HDR_S;


typedef struct tagIapCmdEnterDownloadMode
{
    IAP_CMD_HDR_S head;
    u8 aursv[4]; //Ԥ����ȫ0
    u8 chksum;
    u8 end; // 0x0a
}IAP_CMD_ENTER_DOWNLOAD_S;

extern IAP_FLASH_BUF_S g_stIapBuf;

extern void IAP_Init(void);
extern void IAP_LoadApp(u32 appxaddr);
extern void IAP_SetState(IAP_STATE_E state);
extern inline IAP_STATE_E IAP_GetState(void);
extern void IAP_ResetMCU(void);
extern void IAP_SetIapFlag(uint16_t flag);
extern u16 IAP_ReadIapFlag(void);
extern int IAP_WriteFlashBuf(u8 *buff, u32 len);
extern void IAP_DownloadFlash(void);
extern void IAP_DownloadLastPkgToFlash(void);

#endif

