#include "Support.h"



//UFloatingPointIEEE754 ieee754;

// unsigned int pack754_32 ( float f )
//{
//   unsigned int floatingToIntValue = 0;
//  ieee754.f = f;
//  floatingToIntValue = (((NTH_BIT(ieee754.raw.sign, 0) << 8) |
//  (BYTE_TO_BIN(ieee754.raw.exponent)))  << 23 ) | MANTISSA_TO_BIN(ieee754.raw.mantissa);
//  return floatingToIntValue;
//}

//float unpack754_32( unsigned int floatingToIntValue )
// {
//   UFloatingPointIEEE754 ieee754;
//   unsigned int mantissa = 0;
//   unsigned int exponent = 0 ;
//   unsigned int sign = 0;

//   sign = NTH_BIT(floatingToIntValue, 31);
//   for( int ix=0; ix<8; ix++)
//   exponent = (exponent | (NTH_BIT(floatingToIntValue, (30-ix))))<<1;
//   exponent = exponent>>1;
//   for( int ix=0; ix<23; ix++)
//   mantissa = (mantissa | (NTH_BIT(floatingToIntValue, (22-ix))))<<1;
//   mantissa = mantissa >> 1;

//   ieee754.raw.sign = sign;
//   ieee754.raw.exponent = exponent;
//   ieee754.raw.mantissa = mantissa;
//   return ieee754.f;
// }
static unsigned long reflect(unsigned long data, unsigned char nBits)
{
	unsigned long  reflection = 0x00000000;
	unsigned char  bit;
	for (bit = 0; bit < nBits; ++bit)
	{
		if (data & 0x01)
			reflection |= (1 << ((nBits - 1) - bit));
		data = (data >> 1);
	}
	return (reflection);
}

void crcInit(void)
{
    crc			   remainder;
	int			   dividend;
	unsigned char  bit;

    for (dividend = 0; dividend < 256; ++dividend)
    {
        remainder = dividend << (WIDTH - 8);
        for (bit = 8; bit > 0; --bit)
        {		
            if (remainder & TOPBIT)
                remainder = (remainder << 1) ^ POLYNOMIAL;
            else
                remainder = (remainder << 1);
        }
        crcTable[dividend] = remainder;
    }

} 

crc CRCCALC(unsigned char const message[], int nBytes)
{
    crc	           remainder = INITIAL_REMAINDER;
    unsigned char  data;
	int            byte;

    for (byte = 0; byte < nBytes; ++byte)
    {
        data = REFLECT_DATA(message[byte]) ^ (remainder >> (WIDTH - 8));
  		remainder = crcTable[data] ^ (remainder << 8);
    }
    return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);

} 

unsigned int deserialize_uint32(unsigned char *buf)
{
    unsigned int *x = (unsigned int*)buf;
    return *x;
}

unsigned char * deserialize_uint32B(unsigned char *buffer, unsigned int * value)
{
    *(unsigned int *)buffer = *value;
    return buffer;
}
unsigned short deserialize_uint16(unsigned char *buf)
{
    unsigned short *x = (unsigned short*)buf;
    return *x;
}

unsigned char * serialize_uint16B(unsigned char *buffer, unsigned short * value)
{
    *(unsigned short*)buffer = *value;
    return buffer;
}







