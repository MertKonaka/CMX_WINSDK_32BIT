

#ifndef __Support_H__
#define __Support_H__

#ifdef __cplusplus
    extern "C" {
#endif


#include "cvidef.h"

typedef unsigned long  crc;

#define CRC_NAME			"CRC-32"
#define POLYNOMIAL			0x04C11DB7
#define INITIAL_REMAINDER	0xFFFFFFFF
#define FINAL_XOR_VALUE		0xFFFFFFFF
#define REFLECT_DATA		TRUE
#define REFLECT_REMAINDER	TRUE
#define CHECK_VALUE			0xCBF43926
#define FALSE	0
#define TRUE	!FALSE
#define CRC_CCITT
#define WIDTH    (8 * sizeof(crc))
#define TOPBIT   (1 << (WIDTH - 1))
#if (REFLECT_DATA == TRUE)
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			((unsigned char) reflect((X), 8))
#else
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			(X)
#endif
#if (REFLECT_REMAINDER == TRUE)
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	((crc) reflect((X), WIDTH))
#else
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	(X)
#endif


//#define NTH_BIT(b, n) 		((b >> n) & 0x1)

//#define BYTE_TO_BIN(b)   	(( b & 0x80 ) ) |\
//            				(( b & 0x40 ) ) |\
//            				(( b & 0x20 ) ) |\
//            				(( b & 0x10 ) ) |\
//            				(( b & 0x08 ) ) |\
//            				(( b & 0x04 ) ) |\
//            				(( b & 0x02 ) ) |\
//            				 ( b & 0x01 )

//#define MANTISSA_TO_BIN(b)  (( b & 0x400000 ) ) |\
//             				(( b & 0x200000 ) ) |\
//             				(( b & 0x100000 ) ) |\
//             				(( b &  0x80000 ) ) |\
//             				(( b &  0x40000 ) ) |\
//             				(( b &  0x20000 ) ) |\
//             				(( b &  0x10000 ) ) |\
//             				(( b &  0x8000 ) ) |\
//             				(( b &  0x4000 ) ) |\
//             				(( b &  0x2000 ) ) |\
//             				(( b &  0x1000 ) ) |\
//             				(( b &  0x800 ) ) |\
//             				(( b &  0x400 ) ) |\
//             				(( b &  0x200 ) ) |\
//             				(( b &  0x100 ) ) |\
//             				(( b &  0x80 ) ) |\
//             				(( b &  0x40 ) ) |\
//             				(( b &  0x20 ) ) |\
//             				(( b &  0x10 ) ) |\
//             				(( b &  0x08 ) ) |\
//             				(( b &  0x04 ) ) |\
//             				(( b &  0x02 ) ) |\
//             				 ( b & 0x01 )

// typedef union UnFloatingPointIEEE754
// {
// struct
//  {
//   unsigned int mantissa : 23;
//   unsigned int exponent : 8;
//   unsigned int sign : 1;
//  } raw;
//float f;
//} UFloatingPointIEEE754;

void  crcInit(void);
crc   CRCCALC(unsigned char const message[], int nBytes);
crc  crcTable[256];
//unsigned int pack754_32 ( float f );
//float unpack754_32(unsigned int floatingToIntValue);
unsigned int deserialize_uint32(unsigned char *buf);
unsigned char * deserialize_uint32B(unsigned char *buffer, unsigned int * value);
unsigned short deserialize_uint16(unsigned char *buf);
unsigned char * serialize_uint16B(unsigned char *buffer, unsigned short * value);
#ifdef __cplusplus
    }
#endif

#endif  
