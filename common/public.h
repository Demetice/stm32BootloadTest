#ifndef __PUBLIC__H__
#define __PUBLIC__H__

typedef long LONG;
typedef char CHAR;
typedef short SHORT;
typedef unsigned long ULONG;
typedef unsigned short USHORT;
typedef unsigned char UCHAR;

#define MAX_ULONG 0xffffffff
#define MAX_USHORT 0xffff
#define MAX_UCHAR 0xff


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

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))


typedef enum 
{
    E_RTN_OK = 0,
    E_RTN_NULL_PTR = 1,
    E_RTN_MALLOC_FAIL = 2,
    E_RTN_ERROR_PARAM = 3,
    E_RTN_FATAL_ERROR = 4,
    E_RTN_MSG_ERROR = 5,

    
    E_RTN_BUTT
}RTN_CODE_E;

#define PUB_CHECK_PARAM_AND_RTN(x) if((x))\
{\
    LOGD("Error param");\
    return E_RTN_ERROR_PARAM;\
}

#define PUB_CHECK_RESULT_AND_RTN(rtn, fmt, ...) if((rtn)){\
    LOGD("error code:%d"##fmt, rtn, ##__VA_ARGS__);\
    return rtn;\
}

#define PUB_CHECK_RESULT_AND_RTN_VOID(rtn, fmt, ...) if((rtn)){\
    LOGD("error code:%d"##fmt, rtn, ##__VA_ARGS__);\
    return;\
}

    
#define PUB_CHECK_POINT(rtn) if((rtn)){\
    LOGD("error null point");\
    return E_RTN_NULL_PTR;\
}


#define PUB_CHECK_POINT_RTN_VOID(rtn) if((rtn)){\
    LOGD("error null point", rtn);\
    return;\
}


#endif

