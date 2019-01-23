#ifndef __PUBLIC__H__
#define __PUBLIC__H__

typedef long LONG;
typedef char CHAR;
typedef short SHORT;
typedef unsigned long ULONG;
typedef unsigned short USHORT;
typedef unsigned char UCHAR;



#define LITTLE_ENDIAN (1)
 
#if LITTLE_ENDIAN == 1
 
/* 小端机器要改变字节序 */
#define PUBLIC_HTONL(x) ((0xff000000 & (ULONG)(x)) >> 24 | \
                         (0x00ff0000 & (ULONG)(x)) >> 8  | \
                         (0x0000ff00 & (ULONG)(x)) << 8  | \
                         (0x000000ff & (ULONG)(x)) << 24)
 
#define PUBLIC_NTOHL(x)  ((0xff000000 & (ULONG)(x)) >> 24 | \
                         (0x00ff0000 & (ULONG)(x)) >> 8  | \
                         (0x0000ff00 & (ULONG)(x)) << 8  | \
                         (0x000000ff & (ULONG)(x)) << 24)
 
#define PUBLIC_HTONS(x) ((0xff00 & (SHORT)(x)) >> 8) | ((0xff & (SHORT)(x))<<8)
#define PUBLIC_NTOHS(x) ((0xff00 & (SHORT)(x)) >> 8) | ((0xff & (SHORT)(x))<<8)
 
#else
 
/* 大端机器不用改变字节序 */
#define PUBLIC_HTONL(x) (x)
#define PUBLIC_NTOHL(x) (x)
#define PUBLIC_HTONS(x) (x)
#define PUBLIC_NTOHS(x) (x)
 
#endif


typedef enum 
{
    E_RTN_OK = 0,
    E_RTN_NULL_PTR = 1,
    E_RTN_MALLOC_FAIL = 2,
    E_RTN_ERROR_PARAM = 3,
    E_RTN_FATAL_ERROR = 4,

    
    E_RTN_BUTT
}RTN_CODE_E;

#define PUB_CHECK_PARAM_AND_RTN(x) if((x))\
{\
    LOGD("Error param");\
    return E_RTN_ERROR_PARAM;\
}

#define PUB_CHECK_RESULT_AND_RTN(rtn, fmt, ...) if((rtn)){\
    LOGD("error code:%d"##fmt, rtn, ##__VA_ARGS__);\
    return rtn;
}

#endif

