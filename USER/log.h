#ifndef __LOG_H__0X20190110__
#define __LOG_H__0X20190110__

#include <stdio.h>
#include "FreeRTOS.h"
#include "timers.h"

#define E_DEBUG_LEVEL_DEBUG     1
#define E_DEBUG_LEVEL_WARNNING  2
#define E_DEBUG_LEVEL_ERROR     3
#define E_DEBUG_LEVEL_RELEASE   4

#define __DEBUG_LV__ E_DEBUG_LEVEL_DEBUG

#if (__DEBUG_LV__ <= E_DEBUG_LEVEL_DEBUG)
#define LOGD(fmt, ...) do{\
    printf("[%u D: %s:%d] ", xTaskGetTickCount(), __func__, __LINE__);\
    printf(fmt, ##__VA_ARGS__);\
    printf("\n");\
}while(0)
#else
#define LOGD(fmt, ...) 
#endif

#if (__DEBUG_LV__ <= E_DEBUG_LEVEL_WARNNING)
#define LOGW(fmt, ...) do{\
    printf("[%u W: %s:%d] ", xTaskGetTickCount(), __func__, __LINE__);\
    printf(fmt, ##__VA_ARGS__);\
    printf("\n");\
}while(0)
#else
#define LOGW(fmt, ...) 
#endif

#if (__DEBUG_LV__ <= E_DEBUG_LEVEL_ERROR)
#define LOGE(fmt, ...) do{\
    printf("[%u E: %s:%d] ", xTaskGetTickCount(), __func__, __LINE__);\
    printf(fmt, ##__VA_ARGS__);\
    printf("\n");\
}while(0)
#else
#define LOGE(fmt, ...) 
#endif

#endif

