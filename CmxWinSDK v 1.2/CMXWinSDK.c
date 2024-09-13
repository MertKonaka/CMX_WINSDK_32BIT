//==============================================================================
//
// Title:		CMXAPI
// Purpose:		A short description of the library.
//
// Created on:	17/04/2020 at 11:29:08 by .
// Copyright:	. All Rights Reserved.
//
//==============================================================================

//==============================================================================
// Include files
#include <Windows.h>
#include <tcpsupp.h>
#include <rs232.h>
#include "toolbox.h"
#include <utility.h>
#include <ansi_c.h>
#include <udpsupp.h>
#include "CMXWinSDK.h"
#include "stdio.h"
#include "math.h"
#include "cvidef.h"

// CRC Definitions Begin

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

#define CMX5010 										0x01
#define CMX4010 										0x02
#define CMX6010 										0x03
#define CMX6012 										0x04
#define CMX6012C 										0x05
#define CMX6014											0x06
#define	CMX6020											0x07
#define CMX6022											0x08
#define CMX1553											0x09
#define CMX5015											0x0A
#define CMX4011											0x0B

/*
 * Select the CRC standard from the list that follows.
 */
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

void  crcInit(void);
crc   CRCCALC(unsigned char const message[], int nBytes);

/*********************************************************************
 *
 * Function:    reflect()
 * 
 * Description: Reorder the bits of a binary sequence, by reflecting
 *				them about the middle position.
 *
 * Notes:		No checking is done that nBits <= 32.
 *
 * Returns:		The reflection of the original data.
 *
 *********************************************************************/
static unsigned long reflect(unsigned long data, unsigned char nBits)
{
	unsigned long  reflection = 0x00000000;
	unsigned char  bit;

	/*
	 * Reflect the data about the center bit.
	 */
	for (bit = 0; bit < nBits; ++bit)
	{
		/*
		 * If the LSB bit is set, set the reflection of it.
		 */
		if (data & 0x01)
		{
			reflection |= (1 << ((nBits - 1) - bit));
		}

		data = (data >> 1);
	}

	return (reflection);

}	/* reflect() */

crc  crcTable[256];


/*********************************************************************
 *
 * Function:    crcInit()
 * 
 * Description: Populate the partial CRC lookup table.
 *
 * Notes:		This function must be rerun any time the CRC standard
 *				is changed.  If desired, it can be run "offline" and
 *				the table results stored in an embedded system's ROM.
 *
 * Returns:		None defined.
 *
 *********************************************************************/
void crcInit(void)
{
    crc			   remainder;
	int			   dividend;
	unsigned char  bit;


    /*
     * Compute the remainder of each possible dividend.
     */
    for (dividend = 0; dividend < 256; ++dividend)
    {
        /*
         * Start with the dividend followed by zeros.
         */
        remainder = dividend << (WIDTH - 8);

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (bit = 8; bit > 0; --bit)
        {
            /*
             * Try to divide the current data bit.
             */			
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        /*
         * Store the result into the table.
         */
        crcTable[dividend] = remainder;
    }

}   /* crcInit() */


/*********************************************************************
 *
 * Function:    crcFast()
 * 
 * Description: Compute the CRC of a given message.
 *
 * Notes:		crcInit() must be called first.
 *
 * Returns:		The CRC of the message.
 *
 *********************************************************************/
crc CRCCALC(unsigned char const message[], int nBytes)
{
    crc	           remainder = INITIAL_REMAINDER;
    unsigned char  data;
	int            byte;


    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (byte = 0; byte < nBytes; ++byte)
    {
        data = REFLECT_DATA(message[byte]) ^ (remainder >> (WIDTH - 8));
  		remainder = crcTable[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);

}   /* crcFast() */

// CRC Definitions End

// IEEE 754 Conversion Definitions Begin

#define NTH_BIT(b, n) 		((b >> n) & 0x1)

#define BYTE_TO_BIN(b)   	(( b & 0x80 ) ) |\
            				(( b & 0x40 ) ) |\
            				(( b & 0x20 ) ) |\
            				(( b & 0x10 ) ) |\
            				(( b & 0x08 ) ) |\
            				(( b & 0x04 ) ) |\
            				(( b & 0x02 ) ) |\
            				 ( b & 0x01 )

#define MANTISSA_TO_BIN(b)  (( b & 0x400000 ) ) |\
             				(( b & 0x200000 ) ) |\
             				(( b & 0x100000 ) ) |\
             				(( b &  0x80000 ) ) |\
             				(( b &  0x40000 ) ) |\
             				(( b &  0x20000 ) ) |\
             				(( b &  0x10000 ) ) |\
             				(( b &  0x8000 ) ) |\
             				(( b &  0x4000 ) ) |\
             				(( b &  0x2000 ) ) |\
             				(( b &  0x1000 ) ) |\
             				(( b &  0x800 ) ) |\
             				(( b &  0x400 ) ) |\
             				(( b &  0x200 ) ) |\
             				(( b &  0x100 ) ) |\
             				(( b &  0x80 ) ) |\
             				(( b &  0x40 ) ) |\
             				(( b &  0x20 ) ) |\
             				(( b &  0x10 ) ) |\
             				(( b &  0x08 ) ) |\
             				(( b &  0x04 ) ) |\
             				(( b &  0x02 ) ) |\
             				 ( b & 0x01 )

 typedef union UnFloatingPointIEEE754
 {
 struct
  {
   unsigned int mantissa : 23;
   unsigned int exponent : 8;
   unsigned int sign : 1;
  } raw;   
float f;  
} UFloatingPointIEEE754;

  UFloatingPointIEEE754 ieee754;
  
unsigned int pack754_32 ( float f )
{

  unsigned int    floatingToIntValue = 0;
  ieee754.f = f;
  floatingToIntValue = (((NTH_BIT(ieee754.raw.sign, 0) << 8) |
  (BYTE_TO_BIN(ieee754.raw.exponent)))  << 23 ) | MANTISSA_TO_BIN(ieee754.raw.mantissa);
  return floatingToIntValue;
}

float unpack754_32( unsigned int floatingToIntValue )
 {
   UFloatingPointIEEE754 ieee754;    unsigned int mantissa = 0;
   unsigned int exponent = 0 ;
   unsigned int sign = 0;    
   
   sign = NTH_BIT(floatingToIntValue, 31);
   for( int ix=0; ix<8; ix++)
    exponent = (exponent | (NTH_BIT(floatingToIntValue, (30-ix))))<<1;
   exponent = exponent>>1;
   for( int ix=0; ix<23; ix++)
    mantissa = (mantissa | (NTH_BIT(floatingToIntValue, (22-ix))))<<1;
   mantissa = mantissa >> 1;    
   
   ieee754.raw.sign = sign;
   ieee754.raw.exponent = exponent;
   ieee754.raw.mantissa = mantissa;    
   return ieee754.f;
 }

// IEEE 754 Conversion Definitions End  

// 5010 Protocol Commands Begin
#define DT_OPEN_BOARD						0x01
#define DT_SET_IO_MODE						0x02
#define DT_GET_IO_MODE						0x03
#define DT_SET_THRESHOLD					0x04
#define DT_GET_THRESHOLD					0x05
#define DT_SET_EXCITATION					0x06
#define DT_GET_EXCITATION					0x07
#define DT_SET_STATE						0x08
#define DT_GET_STATE						0x09
#define DT_SET_ALL_STATE					0x0A
#define DT_GET_ALL_STATE					0x0B
#define DT_GET_TIME							0x0C
#define DT_GET_TRANSITION					0x0D
#define DT_GET_ALL_TRANSITION				0x0E
#define DT_GET_CHANNEL_FAULT				0x0F
#define DT_GET_VOLTAGE						0x10
#define DT_RESET_TIME						0x11
#define DT_RESET_FAULTS						0x12
#define DT_RESET_TRANSITIONS				0x13
#define DT_BOARD_RESET						0x14
#define DT_SET_IP							0x15
#define DT_GET_IP							0x16
#define DT_RESET_IP							0x17
#define DT_FIRMWARE_VERSION					0x18
#define DT_SERIAL_NUMBER					0x19
#define DT_SET_BOARD_ID						0x1A
#define DT_GET_BOARD_ID						0x1B
#define DT_RUN_BIT							0x1C
#define DT_GET_BIT_RESULT					0x1D
#define DT_STOP_CONVERSION					0x1E
#define DT_START_CONVERSION					0x1F
#define DT_SET_PWM_FREQUENCY				0x20
#define DT_GET_PWM_FREQUENCY				0x21
#define DT_SET_PWM_DUTY						0x22
#define DT_GET_PWM_DUTY						0x23
#define DT_RUN_PWM							0x24
#define DT_STOP_PWM							0x25
#define DT_SET_VCC_LEVEL					0x26
#define DT_GET_VCC_LEVEL					0x27
#define DT_SET_MAC							0x28  
#define DT_GET_MAC							0x29
#define DT_SET_DEBOUNCE						0x2A
#define DT_GET_DEBOUNCE						0x2B
#define DT_SET_SERIAL						0x2C
#define DT_SET_FW							0x2D
#define DT_RUN_SELF_CALIBRATION				0x2E
#define DT_SET_SCALE_GAIN					0x2F
#define DT_GET_SCALE_GAIN					0x30
#define DT_SET_SCALE_OFFSET					0x31
#define DT_GET_SCALE_OFFSET					0x32
#define DT_GET_ADC_GAIN						0x33
#define DT_GET_ADC_OFFSET					0x34
#define DT_DISCARD_BOARD					0x35
// 5010 Protocol Commands End 

// 6010 Protocol Commands Begin
#define AD_OPEN_BOARD						0x01
#define AD_SET_IO_MODE						0x02
#define AD_GET_IO_MODE						0x03
#define AD_SET_SAMPLE_RATE					0x04
#define AD_GET_SAMPLE_RATE					0x05
#define AD_SET_RANGE						0x06
#define AD_GET_RANGE						0x07
#define AD_SET_CJC_CHANNEL					0x08
#define AD_GET_CJC_CHANNEL					0x09
#define AD_SET_RTD_EXC						0x0A
#define AD_GET_RTD_EXC						0x0B
#define AD_SET_TEMPERATURE_UNIT				0x0C
#define AD_GET_TEMPERATURE_UNIT				0x0D
#define AD_SET_TC_OC						0x0E
#define AD_GET_TC_OC						0x0F
#define AD_GET_CHANNEL_FAULT				0x10
#define AD_GET_CJC_VALUE					0x11
#define AD_SET_FILTER						0x12
#define AD_GET_FILTER						0x13
#define AD_SET_TRIGGER						0x14
#define AD_GET_TRIGGER						0x15
#define AD_GET_ADC_FAULT					0x16
#define AD_SET_FIFO_ENABLE					0x17
#define AD_SET_FIFO_DISABLE					0x18
#define AD_START_CONVERSION					0x19
#define AD_STOP_CONVERSION					0x1A
#define AD_GET_SAMPLE						0x1B
#define AD_GET_ALL_SAMPLES					0x1C 
#define AD_GET_FIFO							0x1D
#define AD_SET_IP							0x1E
#define AD_GET_IP							0x1F
#define AD_RESET_IP							0x20
#define AD_BOARD_RESET						0x21
#define AD_FIRMWARE_VERSION					0x22
#define AD_SERIAL_NUMBER					0x23
#define AD_SET_BOARD_ID						0x24
#define AD_GET_BOARD_ID						0x25 
#define AD_SET_MAC							0x26
#define AD_GET_MAC							0x27
#define AD_SET_SERIAL						0x28
#define AD_SET_FW							0x29
#define AD_RUN_SELF_CALIBRATION				0x2A
#define AD_SET_GAIN							0x2B
#define AD_GET_GAIN							0x2C
#define AD_SET_OFFSET						0x2D
#define AD_GET_OFFSET						0x2E
#define AD_DISCARD_BOARD					0x2F
#define AD_READ_BUFFER						0x30
// 6010 Protocol Commands End

// 4010 Protocol Commands Begin
#define DA_OPEN_BOARD						0x80
#define DA_SET_READBACK						0x81
#define DA_GET_READBACK						0x82
#define DA_SET_CHANNEL_MODE					0x83
#define DA_GET_CHANNEL_MODE					0x84
#define DA_SET_RANGE						0x85
#define DA_GET_RANGE						0x86
#define DA_SET_UPDATE_MODE					0x87
#define DA_GET_UPDATE_MODE					0x88
#define DA_UPDATE_OUTPUT					0x89
#define DA_SET_BROADCAST					0x8A
#define DA_GET_BROADCAST					0x8B
#define DA_SET_BROADCAST_DATA				0x8C
#define DA_GET_BROADCAST_DATA				0x8D
#define DA_SET_GAIN							0x8E
#define DA_GET_GAIN							0x8F
#define DA_SET_OFFSET						0x90
#define DA_GET_OFFSET						0x91
#define DA_START							0x92
#define DA_STOP								0x93
#define DA_SET_CHANNEL_DATA					0x94
#define DA_GET_CHANNEL_DATA					0x95
#define DA_SET_IP							0x96
#define DA_GET_IP							0x97
#define DA_RESET_IP							0x98
#define DA_BOARD_RESET						0x99
#define DA_FIRMWARE_VERSION					0x9A
#define DA_SERIAL_NUMBER					0x9B
#define DA_SET_BOARD_ID						0x9C
#define DA_GET_BOARD_ID						0x9D
#define DA_SET_MAC							0x9E
#define DA_GET_MAC							0x9F
#define DA_SET_SERIAL						0xA0
#define DA_SET_FW							0xA1
#define DA_GET_FAULT						0xA2
#define DA_DISCARD_BOARD					0xA3
#define DA_SET_GENERATION_MODE				0xA4
#define DA_GET_GENERATION_MODE				0xA5
#define DA_SET_BUFFER_SIZE					0xA6
#define DA_GET_BUFFER_SIZE					0xA7
#define DA_WRITE_ARRAY_POINT				0xA8
#define DA_READ_ARRAY_POINT					0xA9
#define DA_SET_UPDATE_TIME					0xAA
// 4010 Protocol Commands End


//1553 Protocol Commands
#define CM_RESET_BOARD						0x33
#define CM_OPEN_BOARD						0x01
#define CM_INIT_BOARD						0x02
#define CM_BOARD_SET_1553_MODE				0x03
#define CM_BOARD_GET_1553_MODE				0x04
#define CM_2130_RESET						0X05
#define CM_SHARED_INIT						0X06
#define CM_BOARD_RAM_TEST					0X07
#define CM_BC_INIT_MSG_DATA					0x08
#define CM_BC_INIT_MSG						0x09
#define CM_BC_INIT_MSG_INST_LIST			0x0A
#define	CM_BC_SET_TTAG						0x0B
#define CM_BC_GET_PROGRESS					0x0C
#define CM_BC_GET_MSG_BLOCK					0x0D
#define CM_BC_GET_MSG_DATA 					0x0E
#define CM_BC_STOP							0x0F
#define CM_BC_ENABLE						0x10
#define CM_BC_START							0x11
#define CM_BC_TRIGGER						0x12
#define CM_MT_SET_TTAG						0x13
#define CM_MT_SET_FILTER					0x14
#define CM_MT_GET_LAST_COMMAND				0x15
#define CM_MT_GET_LAST_MSG					0x16
#define CM_MT_ENABLE						0x17
#define CM_MT_STOP							0x18
#define CM_RT1_INIT							0x19
#define CM_RT1_INIT1						0x20
#define CM_RT1_INIT2						0x21
#define CM_RT1_SET_ADDR						0x22
#define CM_RT1_SET_TTAG						0x23
#define CM_RT1_SET_TX_DATA					0x24
#define CM_RT1_ENABLE						0x25
#define CM_RT1_STOP							0x26
#define CM_RT1_START						0x27
#define CM_RT2_INIT							0x28
#define CM_RT2_INIT1						0x29
#define CM_RT2_INIT2						0x2A
#define CM_RT2_SET_ADDR						0x2B
#define CM_RT2_SET_TTAG						0x2C
#define CM_RT2_SET_TX_DATA					0x2D
#define CM_RT2_ENABLE						0x2E
#define CM_RT2_STOP							0x2F
#define CM_RT2_START						0x30
#define CM_RT_GET_MSG						0x31
#define CM_RT_CHECK_ADDR					0x32
#define CM_DISCARD_BOARD					0x34
// 1553 Protocol Commands End

// 6022 Protocol Commands Begin
#define TC_OPEN_BOARD						0x01
#define TC_SET_TYPE 						0x02
#define TC_GET_TYPE							0x03
#define TC_SET_CJC_CHANNEL					0x04
#define TC_GET_CJC_CHANNEL					0x05
#define TC_SET_TEMPERATURE_UNIT				0x06
#define TC_GET_TEMPERATURE_UNIT				0x07
#define TC_SET_TRIGGER						0x08
#define TC_GET_TRIGGER						0x09
#define TC_START_CONVERSION					0x0A
#define TC_STOP_CONVERSION					0x0B
#define TC_READ_BUFFER						0x0C
#define TC_SET_CJC_VALUE					0x0D
#define TC_GET_CJC_VALUE					0x0E
#define TC_SET_GAIN							0x0F
#define TC_GET_GAIN							0x10
#define TC_SET_OFFSET						0x11
#define TC_GET_OFFSET						0x12
#define TC_SET_TEMPVAL						0x13
#define TC_SET_TEMP_GAIN					0x14
#define TC_SET_TEMP_OFFSET					0x15
#define TC_DISCARD_BOARD					0x16

#define CHASSIS_SET_SUBMASK					0xE0
#define CHASSIS_GET_SUBMASK					0xE1
#define CHASSIS_SET_GATEWAY					0xE2
#define CHASSIS_GET_GATEWAY					0xE3
#define OPEN_CHASSIS						0xF0
#define CHASSIS_SET_IP						0xF1
#define CHASSIS_GET_IP						0xF2
#define CHASSIS_SET_MODULE					0xF3
#define CHASSIS_GET_MODULE					0xF4
#define CHASSIS_GET_DEP_STATUS				0xF5
#define DISCARD_CHASSIS						0xFF

#define REBOOT_SYSTEM						0x01

#define ETHERNET							0x00
#define USB									0x01
#define SERIAL_485							0x02

#define VERSION								"1.2"  

int iteration_count = 3;

//==============================================================================
// Types

//==============================================================================
// Static global variables

unsigned char BOARD_ID_ERROR[2] = {0x02, 0x00};
unsigned char CRC_ERROR[2] = { 0x03, 0x00 };
unsigned char TIMEOUT_ERR[2] = {0x04, 0x00};
unsigned char INTERFACE_ERROR[2] = {0x05, 0x00};
unsigned char INVALID_PARAMETER_ERROR[2] = {0x06, 0x00};
unsigned char UNSUPPORTED_FUNCTION[2] = {0x07, 0x00};
unsigned char NO_CONNECTION_ERROR[2] = {0x08, 0x00};
unsigned char COMMAND_SEND_ERROR[2] = {0x09, 0x00};
unsigned char GENERIC_ERROR[2] = {0x80, 0x00};
unsigned char NO_ERR[2] = {0x01, 0x00};
unsigned int DT_REMOTE_PORT	= 59000; 
unsigned int AD_REMOTE_PORT = 59000;
unsigned int DA_REMOTE_PORT = 59000;
unsigned int TIMEOUT = 2000;
unsigned int BAUD_RATE = 115200;//115200;
double _temperature;
#define RebootPort				54990
#define DebugPort				54991
//==============================================================================
// Static functions
int itoa(int value,char *ptr); 
unsigned int deserialize_uint32(unsigned char *buf);
unsigned char * deserialize_uint32B(unsigned char *buffer, unsigned int * value);
uint16_t deserialize_uint16(unsigned char *buf);
uint8_t * serialize_uint16B(uint8_t *buffer, uint16_t * value);
double BINARY2UNIPOLARVOLT(unsigned int BINARY);
double BINARY2BIPOLARVOLT(unsigned int BINARY);
double BINARY2UNIPOLARCURRENT(unsigned int BINARY);
double BINARY2BIPOLARCURRENT(unsigned int BINARY);
double BINARY2TEMPERATURE(unsigned int BINARY);
//==============================================================================
// Global variables
unsigned int DestPort(unsigned int BoardId);
unsigned int DestPort (unsigned int BoardId)
{
	return 55000 + (BoardId * 100);
}
void waitMs(void)
{
	Sleep(10);	
}
//==============================================================================
// Global functions
/// HIFN  What does your function do?
/// HIPAR x/What inputs does your function expect?
/// HIRET What does your function return?
unsigned int CmxDTOpenBoard (char* IPAddress, unsigned int BoardID, unsigned int BoardReset, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14]={0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned int Handle = 0;	
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_OPEN_BOARD;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	int tcp_error = 0;
	unsigned int port = DestPort(BoardID);
	tcp_error = ConnectToTCPServerEx (&Handle, port, IPAddress, 0, 0, 2000, TCP_ANY_LOCAL_PORT);
	if (tcp_error < 0)
		memcpy(error_temp, NO_CONNECTION_ERROR, 2); 
	else
	{	
		if (0)
		{
			//for (int iteration = 0; iteration < iteration_count; iteration++)
			{
				int messageSize = 14;
				int bytesToWrite = 14;
				int bytesWritten = 0;
				while (bytesToWrite > 0)
				{
					bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
					if (bytesWritten>=0)
						bytesToWrite -= bytesWritten;
					else
						break;
				}
				if (bytesWritten < 0)
					memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
				else
				{
					unsigned int bytestoread = 14;
					int byteread = 0;
					messageSize = 14;
					int totalbytes = 0;
					while (totalbytes < 13)
					{
						byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
						if (byteread > 0)
						{
							bytestoread -= byteread;
							totalbytes = byteread + totalbytes;
						}
						else if (byteread < 0)
							break;
					}
					if ((byteread < 0) || (totalbytes < 14))
						memcpy(error_temp,TIMEOUT_ERR,2); 
					else
					{
						memcpy(crc_temp,response,10);
						memcpy(crc_array,response+10,4);
						unsigned int crc_result = CRCCALC(crc_temp,10);
						if (crc_result == deserialize_uint32(crc_array))
						{
							memcpy(error_temp,response+8,2);
							//break;
						}
						else
							memcpy(error_temp,CRC_ERROR,2);
					}
				}
			}
		}
		else
		{
			error_temp[0] = 0x01;
			error_temp[1] = 0x00;
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	Sleep(500);
	return Handle;
}

unsigned int CmxDTSetIOMode(unsigned int Handle, unsigned int Channel, unsigned int Mode, unsigned int *Error)
{	

	unsigned char TX_COMMAND[14];
	unsigned char  response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;//BoardID
	TX_COMMAND[1] = DT_SET_IO_MODE;
	TX_COMMAND[2] = (unsigned char)Channel;		// CHANNEL: 0-7
	TX_COMMAND[3] = (unsigned char)Mode;		// Mode 0: O/28 In, Mode 1: O/G In, Mode 2: High Side Output, Mode 3: Low Side Output, 4: PWM Output
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	for (int iteration = 0; iteration < iteration_count; iteration++)
	{
		int messageSize = 14;
		int bytesToWrite = 14;
		int bytesWritten = 0;
		while (bytesToWrite > 0)
		{
			bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
			if (bytesWritten>=0)
				bytesToWrite -= bytesWritten;
			else
				break;
		}
		if (bytesWritten < 0)
			memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
		else
		{
			unsigned int bytestoread = 14;
			int byteread = 0;
			messageSize = 14;
			int totalbytes = 0;
			while (totalbytes < 13)
			{
				byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
				if (byteread > 0)
				{
					bytestoread -= byteread;
					totalbytes = byteread + totalbytes;
				}
				else if (byteread < 0)
					break;
			}
			if ((byteread < 0) || (totalbytes < 14))
				memcpy(error_temp,TIMEOUT_ERR,2); 
			else
			{
				memcpy(crc_temp,response,10);
				memcpy(crc_array,response+10,4);
				unsigned int crc_result = CRCCALC(crc_temp,10);
				if (crc_result == deserialize_uint32(crc_array))
				{
					memcpy(error_temp,response+8,2);
					break;
				}
				else
					memcpy(error_temp,CRC_ERROR,2);
			}
		}	
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetIOMode(unsigned int Handle, unsigned int Channel, unsigned int *IOMode, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_IO_MODE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	for (int iteration = 0; iteration < iteration_count; iteration++)
	{
		int messageSize = 14;
		int bytesToWrite = 14;
		int bytesWritten = 0;
		while (bytesToWrite > 0)
		{
			bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
			if (bytesWritten>=0)
				bytesToWrite -= bytesWritten;
			else
				break;
		}
		if (bytesWritten < 0)
			memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
		else
		{
			unsigned int bytestoread = 14;
			int byteread = 0;
			messageSize = 14;
			int totalbytes = 0;
			while (totalbytes < 13)
			{
				byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
				if (byteread > 0)
				{
					bytestoread -= byteread;
					totalbytes = byteread + totalbytes;
				}
				else if (byteread < 0)
					break;
			}
			if ((byteread < 0) || (totalbytes < 14))
				memcpy(error_temp,TIMEOUT_ERR,2); 
			else
			{
				memcpy(crc_temp,response,10);
				memcpy(crc_array,response+10,4);
				unsigned int crc_result = CRCCALC(crc_temp,10);
				if (crc_result == deserialize_uint32(crc_array))
				{
					*IOMode = (unsigned int)response[2]; 
					memcpy(error_temp,response+8,2);
					break;
				}
				else
					memcpy(error_temp,CRC_ERROR,2);
			}
		}	
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTSetThreshold(unsigned int Handle, unsigned int Channel, unsigned int Type, double Threshold, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_THRESHOLD;
	TX_COMMAND[2] = (unsigned char)Channel;		// CHANNEL: 0-7
	TX_COMMAND[3] = (unsigned char)Type;		//	0: MIN_LOW_THRES, 1:LOWER_THRES, 2:UPPER_THRES, 3:MAX_HI_THRES
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	unsigned int threshold_bin = pack754_32(Threshold);
	unsigned char threshold_bin_array[4] = {0x00, 0x00, 0x00, 0x00};
	deserialize_uint32B(threshold_bin_array,&threshold_bin);
	memcpy(TX_COMMAND+4,threshold_bin_array,4);
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	for (int iteration = 0; iteration < iteration_count; iteration++)
	{
		int messageSize = 14;
		int bytesToWrite = 14;
		int bytesWritten = 0;
		while (bytesToWrite > 0)
		{
			bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
			if (bytesWritten>=0)
				bytesToWrite -= bytesWritten;
			else
				break;
		}
		if (bytesWritten < 0)
			memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
		else
		{
			unsigned int bytestoread = 14;
			int byteread = 0;
			messageSize = 14;
			int totalbytes = 0;
			while (totalbytes < 13)
			{
				byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
				if (byteread > 0)
				{
					bytestoread -= byteread;
					totalbytes = byteread + totalbytes;
				}
				else if (byteread < 0)
					break;
			}
			if ((byteread < 0) || (totalbytes < 14))
				memcpy(error_temp,TIMEOUT_ERR,2); 
			else
			{
				memcpy(crc_temp,response,10);
				memcpy(crc_array,response+10,4);
				unsigned int crc_result = CRCCALC(crc_temp,10);
				if (crc_result == deserialize_uint32(crc_array))
				{
					memcpy(error_temp,response+8,2);
					break;
				}
				else
					memcpy(error_temp,CRC_ERROR,2);
			}
		}	
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetThreshold(unsigned int Handle, unsigned int Channel, unsigned int Type, double *Threshold, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char threshold_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_THRESHOLD;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char)Type;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	for (int iteration = 0; iteration < iteration_count; iteration++)
	{
		int messageSize = 14;
		int bytesToWrite = 14;
		int bytesWritten = 0;
		while (bytesToWrite > 0)
		{
			bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
			if (bytesWritten>=0)
				bytesToWrite -= bytesWritten;
			else
				break;
		}
		if (bytesWritten < 0)
			memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
		else
		{
			unsigned int bytestoread = 14;
			int byteread = 0;
			messageSize = 14;
			int totalbytes = 0;
			while (totalbytes < 13)
			{
				byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
				if (byteread > 0)
				{
					bytestoread -= byteread;
					totalbytes = byteread + totalbytes;
				}
				else if (byteread < 0)
					break;
			}
			if ((byteread < 0) || (totalbytes < 14))
				memcpy(error_temp,TIMEOUT_ERR,2); 
			else
			{
				memcpy(crc_temp,response,10);
				memcpy(crc_array,response+10,4);
				unsigned int crc_result = CRCCALC(crc_temp,10);
				if (crc_result == deserialize_uint32(crc_array))
				{
					memcpy(error_temp,response+8,2);
					memcpy(threshold_bin_array,response+2,4);
					unsigned int threshold_bin = deserialize_uint32(threshold_bin_array);
					*Threshold = unpack754_32(threshold_bin);
					break;
				}
				else
					memcpy(error_temp,CRC_ERROR,2);
			}
		}	
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTSetExcitation(unsigned int Handle, unsigned int Channel, unsigned int State, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_EXCITATION;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char)State;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetExcitation(unsigned int Handle, unsigned int Channel, unsigned int *State, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_EXCITATION;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*State = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTSetState(unsigned int Handle, unsigned int Channel, unsigned int State, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_STATE;
	TX_COMMAND[2] = (unsigned char)Channel;		// CHANNEL: 0-7
	TX_COMMAND[3] = (unsigned char)State;		// 0: OFF, 1: ON
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetState(unsigned int Handle, unsigned int Channel, unsigned int *State, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_STATE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*State = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTSetAllState(unsigned int Handle, unsigned int AllState, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_ALL_STATE;
	TX_COMMAND[2] = (unsigned char)AllState;	
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetAllState(unsigned int Handle, unsigned int *AllState, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_ALL_STATE;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		//Sleep(100);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*AllState = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetTime(unsigned int Handle, unsigned int *Time, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char time_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_TIME;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4); 
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(time_array,response+2,4);
				unsigned int time_ = deserialize_uint32(time_array);
				*Time = time_;
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetTransition(unsigned int Handle, unsigned int Channel, unsigned int Type, unsigned int *Transition, unsigned int *Time, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char time_[4] ={0};
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_TRANSITION;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char)Type;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*Transition = (unsigned int)response[2]; 
				memcpy(time_,response+3,4);
				*Time = (unsigned int) deserialize_uint32(time_);
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetAllTransition(unsigned int Handle, unsigned int Type, unsigned int *AllTransition, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_ALL_TRANSITION;
	TX_COMMAND[2] = (unsigned char)Type;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*AllTransition = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;	
}

unsigned int CmxDTGetChannelFault(unsigned int Handle, unsigned int Channel, unsigned int *Fault, unsigned int *Time, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_CHANNEL_FAULT;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	unsigned char time_[4] = {0};
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*Fault = (unsigned int)response[2]; 
				memcpy(time_,response+3,4);
				*Time = (unsigned int) deserialize_uint32(time_);
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetVoltage(unsigned int Handle, unsigned int Channel, double *Voltage, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char voltage_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_VOLTAGE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(voltage_bin_array,response+2,4);
				unsigned int voltage_bin = deserialize_uint32(voltage_bin_array);
				*Voltage = unpack754_32(voltage_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTResetTime(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_RESET_TIME;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTResetFaults(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_RESET_FAULTS;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTResetTransitions(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_RESET_TRANSITIONS;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTBoardReset(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_BOARD_RESET;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTSetIP(unsigned int Handle, unsigned int NewIpAddress[4], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_IP;
	TX_COMMAND[2] = (unsigned char)NewIpAddress[0];
	TX_COMMAND[3] = (unsigned char)NewIpAddress[1];
	TX_COMMAND[4] = (unsigned char)NewIpAddress[2];
	TX_COMMAND[5] = (unsigned char)NewIpAddress[3];
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetIP(unsigned int Handle, unsigned int IpAddressReturn[], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_IP;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				unsigned char ip_array[4] = { 0x00, 0x00, 0x00, 0x00 };
				memcpy(ip_array,response+2,4);
				IpAddressReturn[0] = (unsigned int)ip_array[0];
				IpAddressReturn[1] = (unsigned int)ip_array[1]; 
				IpAddressReturn[2] = (unsigned int)ip_array[2]; 
				IpAddressReturn[3] = (unsigned int)ip_array[3]; 
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTResetIP(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_RESET_IP;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTFirmwareVersion(unsigned int Handle, double *FWVersion, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_FIRMWARE_VERSION;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				unsigned char fw_array[4] = { 0x00, 0x00, 0x00, 0x00 };
				memcpy(error_temp,response+8,2);
				memcpy(fw_array,response+2,4);
				unsigned int fw_ = deserialize_uint32(fw_array);
				*FWVersion = unpack754_32(fw_);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTBoardSerialNumber(unsigned int Handle, unsigned int *SerialNo, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char serial_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SERIAL_NUMBER;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(serial_array,response+2,4);
				unsigned int serial_ = deserialize_uint32(serial_array);
				*SerialNo = serial_;
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTSetBoardID(unsigned int Handle, unsigned int NewBoardID, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_BOARD_ID;
	TX_COMMAND[2] = (unsigned char)NewBoardID;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetBoardID(unsigned int Handle, unsigned int *ReturnBoardID, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_BOARD_ID;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				unsigned char id_array[4] = { 0x00, 0x00, 0x00, 0x00};
				memcpy(id_array,response+2,4);
				unsigned int id_u32 = deserialize_uint32(id_array);
				*ReturnBoardID = (unsigned int)id_u32;
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTRunBIT(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_RUN_BIT;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetBITResult(unsigned int Handle, unsigned int *BITResult, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_BIT_RESULT;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*BITResult = (unsigned int)response[2];
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTStopConversion(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_STOP_CONVERSION;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTStartConversion(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_START_CONVERSION;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTSetPWMFrequency(unsigned int Handle, unsigned int Channel, unsigned int PWMFrequency, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_PWM_FREQUENCY;
	TX_COMMAND[2] = (unsigned char)Channel;
	unsigned char freq_array[4] = {0x00, 0x00, 0x00, 0x00};
	deserialize_uint32B(freq_array,&PWMFrequency);
	TX_COMMAND[3] = freq_array[0];
	TX_COMMAND[4] = freq_array[1];
	TX_COMMAND[5] = freq_array[2];
	TX_COMMAND[6] = freq_array[3];
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetPWMFrequency(unsigned int Handle, unsigned int Channel, unsigned int *PWMFrequency, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_PWM_FREQUENCY;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				unsigned char freq_array[4] = {0x00,0x00,0x00,0x00};
				memcpy(freq_array,response+2,4);
				*PWMFrequency = (unsigned int)deserialize_uint32(freq_array);
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTSetPWMDuty(unsigned int Handle, unsigned int Channel, double PWMDuty, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_PWM_DUTY;
	TX_COMMAND[2] = (unsigned char)Channel;
	unsigned char duty_array[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned int duty_u32 = pack754_32(PWMDuty);
	deserialize_uint32B(duty_array,&duty_u32);
	TX_COMMAND[3] = duty_array[0];
	TX_COMMAND[4] = duty_array[1];
	TX_COMMAND[5] = duty_array[2];
	TX_COMMAND[6] = duty_array[3];
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
 
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetPWMDuty(unsigned int Handle, unsigned int Channel, double *PWMDuty, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_PWM_DUTY;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				unsigned char duty_array[4] = {0x00, 0x00, 0x00, 0x00};
				memcpy(duty_array,response+2,4);
				unsigned int duty_u32 = deserialize_uint32(duty_array);
				*PWMDuty = unpack754_32(duty_u32);
				memcpy(error_temp,response+8,2); 
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTRunPWM(unsigned int Handle, unsigned int Channel, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};   	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_RUN_PWM;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTStopPWM(unsigned int Handle, unsigned int Channel, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_STOP_PWM;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTSetVccLevel(unsigned int Handle, double VccLevel, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_VCC_LEVEL;
	unsigned char level_array[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned int level_u32 = pack754_32(VccLevel);
	deserialize_uint32B(level_array,&level_u32);
	TX_COMMAND[2] = level_array[0];
	TX_COMMAND[3] = level_array[1];
	TX_COMMAND[4] = level_array[2];
	TX_COMMAND[5] = level_array[3];
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTGetVccLevel(unsigned int Handle, double *VccLevel, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_VCC_LEVEL;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				unsigned char level_array[4] = {0x00, 0x00, 0x00, 0x00};
				memcpy(level_array,response+2,4);
				unsigned int level_u32 = deserialize_uint32(level_array);
				*VccLevel =	unpack754_32(level_u32);
				memcpy(error_temp,response+8,2);   
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTSetMAC(unsigned int Handle, unsigned int NewMAC[6], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_MAC;
	TX_COMMAND[2] = (unsigned char)NewMAC[0];
	TX_COMMAND[3] = (unsigned char)NewMAC[1];
	TX_COMMAND[4] = (unsigned char)NewMAC[2];
	TX_COMMAND[5] = (unsigned char)NewMAC[3];
	TX_COMMAND[6] = (unsigned char)NewMAC[4];
	TX_COMMAND[7] = (unsigned char)NewMAC[5];
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTGetMAC(unsigned int Handle, unsigned int ReturnMac[], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_MAC;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				unsigned char mac_array[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
				memcpy(mac_array,response+2,6);
				ReturnMac = malloc(6);
				ReturnMac[0] = (unsigned int)mac_array[0];
				ReturnMac[1] = (unsigned int)mac_array[1]; 
				ReturnMac[2] = (unsigned int)mac_array[2]; 
				ReturnMac[3] = (unsigned int)mac_array[3]; 
				ReturnMac[4] = (unsigned int)mac_array[4];
				ReturnMac[5] = (unsigned int)mac_array[5];
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTSetDebounce(unsigned int Handle, unsigned int Channel, unsigned int DebounceTime, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_DEBOUNCE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char)DebounceTime;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTGetDebounce(unsigned int Handle, unsigned int Channel, unsigned int *DebounceTime, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_DEBOUNCE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*DebounceTime = (unsigned int)response[2];
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTSetSerial(unsigned int Handle, unsigned int SerialNo, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_SERIAL;
	unsigned char serial_array[4] = {0x00, 0x00, 0x00, 0x00};
	deserialize_uint32B(serial_array,&SerialNo);
	TX_COMMAND[2] = (unsigned char)serial_array[0];
	TX_COMMAND[3] = (unsigned char)serial_array[1];
	TX_COMMAND[4] = (unsigned char)serial_array[2];
	TX_COMMAND[5] = (unsigned char)serial_array[3];
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTSetFirmware(unsigned int Handle, double Firmware, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_FW;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	unsigned int fw_bin = pack754_32(Firmware);
	unsigned char fw_array[4] = {0x00, 0x00, 0x00, 0x00} ;
	deserialize_uint32B(fw_array,&fw_bin);
	memcpy(TX_COMMAND+2,fw_array,4);
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTRunSelfCalibration(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_RUN_SELF_CALIBRATION;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTSetScaleGain(unsigned int Handle, unsigned int Channel, double ScaleGain, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_SCALE_GAIN;
	TX_COMMAND[2] = (unsigned char) Channel;
	unsigned char level_array[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned int level_u32 = pack754_32(ScaleGain);
	deserialize_uint32B(level_array,&level_u32);
	TX_COMMAND[3] = level_array[0];
	TX_COMMAND[4] = level_array[1];
	TX_COMMAND[5] = level_array[2];
	TX_COMMAND[6] = level_array[3];
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTGetScaleGain(unsigned int Handle, unsigned int Channel, double *ScaleGain, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char scale_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_SCALE_GAIN;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(scale_bin_array,response+2,4);
				unsigned int gain_bin = deserialize_uint32(scale_bin_array);
				*ScaleGain = unpack754_32(gain_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTSetScaleOffset(unsigned int Handle, unsigned int Channel, double ScaleOffset, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_SET_SCALE_OFFSET;
	TX_COMMAND[2] = (unsigned char) Channel;
	unsigned char level_array[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned int level_u32 = pack754_32(ScaleOffset);
	deserialize_uint32B(level_array,&level_u32);
	TX_COMMAND[3] = level_array[0];
	TX_COMMAND[4] = level_array[1];
	TX_COMMAND[5] = level_array[2];
	TX_COMMAND[6] = level_array[3];
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTGetScaleOffset(unsigned int Handle, unsigned int Channel, double *ScaleOffset, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char scale_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_SCALE_OFFSET;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(scale_bin_array,response+2,4);
				unsigned int offset_bin = deserialize_uint32(scale_bin_array);
				*ScaleOffset = unpack754_32(offset_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTGetADCGain(unsigned int Handle, unsigned int Channel, unsigned int *ADCGain, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_ADC_GAIN;
	TX_COMMAND[2] = (unsigned char) Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				unsigned char adc_gain[4] = { 0x00, 0x00, 0x00, 0x00 };
				memcpy(adc_gain,response+2,4);
				*ADCGain = (unsigned int) deserialize_uint32(adc_gain);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDTGetADCOffset(unsigned int Handle, unsigned int Channel, unsigned int *ADCOffset, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_GET_ADC_OFFSET;
	TX_COMMAND[2] = (unsigned char) Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				unsigned char adc_offset[4] = { 0x00, 0x00, 0x00, 0x00 };
				memcpy(adc_offset,response+2,4);
				*ADCOffset = (unsigned int) deserialize_uint32(adc_offset);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDTDiscardBoard(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DT_DISCARD_BOARD;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		error_temp[0] = 1;
		error_temp[1] = 0;
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	DisconnectFromTCPServer (Handle);
	return 0;	
}

/* 	Discrete IO Board Functions END 	*/

/*	Analog Input Board Functions BEGIN	*/
unsigned int CmxADOpenBoard (char* IPAddress, unsigned int BoardID, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14]={0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned int Handle = 0;	
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_OPEN_BOARD;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	int tcp_error = 0;
	unsigned int port = DestPort(BoardID);
	tcp_error = ConnectToTCPServerEx (&Handle, port, IPAddress, 0, 0, 2000, TCP_ANY_LOCAL_PORT);
	if (tcp_error < 0)
		memcpy(error_temp, NO_CONNECTION_ERROR, 2); 
	else
	{	
		for (int iteration = 0; iteration < 3; iteration++)
		{
		int messageSize = 14;
		int bytesToWrite = 14;
		int bytesWritten = 0;
		while (bytesToWrite > 0)
		{
			bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
			if (bytesWritten>=0)
				bytesToWrite -= bytesWritten;
			else
				break;
		}
		if (bytesWritten < 0)
			memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
		else
		{
			unsigned int bytestoread = 14;
			int byteread = 0;
			messageSize = 14;
			int totalbytes = 0;
			while (totalbytes < 13)
			{
				byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
				if (byteread > 0)
				{
					bytestoread -= byteread;
					totalbytes = byteread + totalbytes;
				}
				else if (byteread < 0)
					break;
			}
			if ((byteread < 0) || (totalbytes < 14))
				memcpy(error_temp,TIMEOUT_ERR,2); 
			else
			{
				memcpy(crc_temp,response,10);
				memcpy(crc_array,response+10,4);
				unsigned int crc_result = CRCCALC(crc_temp,10);
				if (crc_result == deserialize_uint32(crc_array))
				{
					memcpy(error_temp,response+8,2);
					break;
				}
				else
					memcpy(error_temp,CRC_ERROR,2);
			}
		}	
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	Sleep(1000);
	return Handle;
}

unsigned int CmxADSetIOMode (unsigned int Handle, unsigned int Channel, unsigned int Mode, unsigned int InputConfiguration, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_SET_IO_MODE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char)Mode;
	TX_COMMAND[4] = (unsigned char)InputConfiguration;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	for (int iteration = 0; iteration < 3; iteration++)
	{
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				break;
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	
	return Handle;
}

unsigned int CmxADGetIOMode (unsigned int Handle, unsigned int Channel, unsigned int *IOMode, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_GET_IO_MODE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*IOMode = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADSetSampleRate (unsigned int Handle, unsigned int SampleRate, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_SET_SAMPLE_RATE;
	TX_COMMAND[2] = (unsigned char)SampleRate;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADGetSampleRate (unsigned int Handle, unsigned int *SampleRate, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_GET_SAMPLE_RATE;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*SampleRate = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADSetRange (unsigned int Handle, unsigned int Channel, unsigned int Range, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_SET_RANGE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char)Range;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				{
					memcpy(error_temp,response+8,2);
				}
				else
					memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADGetRange (unsigned int Handle, unsigned int Channel, unsigned int *Range, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_GET_RANGE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*Range = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADSetTrigger (unsigned int Handle, unsigned int TriggerType, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_SET_TRIGGER;
	TX_COMMAND[2] = (unsigned char)TriggerType;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADGetTrigger (unsigned int Handle, unsigned int *TriggerType, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_GET_TRIGGER;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
 
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*TriggerType = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADStartConversion (unsigned int Handle, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_START_CONVERSION;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	
	return Handle;
}

unsigned int CmxADGetSample (unsigned int Handle, unsigned int Channel, double *Sample, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char voltage_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_GET_SAMPLE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
 
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(voltage_bin_array,response+2,4);
				unsigned int voltage_bin = deserialize_uint32(voltage_bin_array);
				*Sample = unpack754_32(voltage_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADGetAllSample (unsigned int Handle, double Sample[16], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_result = 0;
	unsigned int crc_u32=0;
	unsigned char voltage_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_GET_ALL_SAMPLES;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4] = {0, 0, 0, 0};
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 72;
		int byteread = 0;
		messageSize = 72;
		int totalbytes = 0;
		while (totalbytes < 71)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (72 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 72))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy (crc_array, response+68, 4);
			crc_result = CRCCALC (response,68);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy (error_temp, response+66, 2);
				unsigned int data_bin = 0;
				for (int i = 0; i<=15; i++)
				{
					memcpy (voltage_bin_array, response+2+(i*4), 4);
					data_bin = deserialize_uint32 (voltage_bin_array);
					Sample[i] = unpack754_32 (data_bin); 
				}
			}
			else
				memcpy (error_temp, CRC_ERROR, 2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADReadBuffer (unsigned int Handle, unsigned int TimeOut, unsigned int NumberOfSamples, double *Buffer, unsigned int *FetchedSampleCount, unsigned int *Error)
{
	//int byteread = 0;
	//unsigned char temp[4] = {0, 0, 0, 0};
	//unsigned int u32 = 0;
	//unsigned int messagesize = NumberOfSamples;
	//unsigned int sample_size = NumberOfSamples;
	//messagesize = messagesize * 32;
	//sample_size = sample_size * 8;
	///*unsigned char *buffer_[messagesize];*/
	//unsigned char *buffer_;
	//buffer_ = malloc(messagesize);
	//
	//unsigned int bytestoread = messagesize;
	//int totalbytes = 0;
	//unsigned char response[10000]={0};
	//unsigned int byte_count = NumberOfSamples * 32;
	//bytestoread = byte_count;
	//byteread = 0;
	//totalbytes = 0;
	
	//while (totalbyte < messagesize)
	//{
	//	byteread = ClientTCPRead (Handle, &buffer_[messagesize - bytestoread], bytestoread, 0);
	//	if (byteread >=0)
	//	{
	//		bytestoread -= byteread;
	//		totalbyte = byteread + totalbyte;
	//	}
	//	if (byteread < 0)
	//		break;
	//}
	//if (totalbyte > 0)
	//{
	//	for (int i = 0; i < sample_size; i++)
	//	{
	//		memcpy (temp, buffer_ + (i * 4), 4);
	//		u32 = deserialize_uint32(temp);
	//		Buffer[i] = unpack754_32(u32);
	//	}
	//	*Error = 0x01;
	//}
	//else
	//	*Error = 0x04;	
	//*FetchedSampleCount = (totalbyte / 4);
	//free(buffer_);
	
	unsigned int bytestoread = 0;
	int byteread = 0;
	unsigned int messageSize = NumberOfSamples * 32;
	int totalbytes = 0;
	unsigned char *response;
	response = malloc(messageSize);
	unsigned int byte_count = NumberOfSamples * 32;
	unsigned int sample_size = NumberOfSamples * 8;
	unsigned char temp[4] = {0, 0, 0, 0};
	unsigned int u32 = 0;
	bytestoread = byte_count;
	byteread = 0;
	
	totalbytes = 0;
	while (totalbytes < byte_count-1)
	{
		byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (byte_count - totalbytes), TimeOut);
		if (byteread > 0)
		{
			bytestoread -= byteread;
			totalbytes = byteread + totalbytes;
		}
		else if (byteread < 0)
		{
			Sleep(TimeOut);
			break;
		}
	}
		
	if (totalbytes > 0)
	{
		for (int i = 0; i < sample_size; i++)
		{
			memcpy (temp, response + (i * 4), 4);
			u32 = deserialize_uint32(temp);
			Buffer[i] = unpack754_32(u32);
		}
		*Error = 0x01;
	}
	else
		*Error = 0x04;	
	*FetchedSampleCount = (totalbytes / 4);	
	free(response);
	return Handle;
}

unsigned int CmxADStopConversion (unsigned int Handle, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_STOP_CONVERSION;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
		{
			memcpy(error_temp,TIMEOUT_ERR,2); 
		}
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADDiscardBoard (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_DISCARD_BOARD;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		error_temp[0] = 1;
		error_temp[1] = 0;
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	DisconnectFromTCPServer (Handle);
	return 0;
}

unsigned int CmxFlushBuffer(unsigned int Handle, unsigned int *Error)
{
	char temp = 0;
	int return_error = 1;
	while (return_error >= 0)
		return_error = ClientTCPRead (Handle, &temp, 1, 1);
	return 0;
}

//unsigned int CmxADSetCjCChannel (unsigned int Handle, unsigned int CjcChannel, unsigned int *Error)
//{	
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = 0x01;
//	TX_COMMAND[1] = AD_SET_CJC_CHANNEL;
//	TX_COMMAND[2] = (unsigned char)CjcChannel;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	response = malloc(RX_SIZE); 
//	int tcp_error = 0;
//	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
//	if (tcp_error < 0)
//		memcpy(error_temp,COMMAND_SEND_ERROR,2);
//	else
//	{	
//		unsigned int bytestoread = 14;
//		int byteread = 0;
//		unsigned int totalbyte = 0;
//		unsigned int messagesize = 14;
//		clock_t start_time = clock();
//		while (clock() < start_time + TIMEOUT)
//		{
//			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
//			waitMs();
//			if (byteread >=0)
//			{
//				bytestoread -= byteread;
//				totalbyte = byteread + totalbyte;
//			}
//			if (totalbyte == messagesize)
//			break;
//		}
//		if (totalbyte <= 0)
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			memcpy(crc_temp,response,10);
//			memcpy(crc_array,response+10,4);
//			unsigned int crc_result = CRCCALC(crc_temp,10);
//			if (crc_result == deserialize_uint32(crc_array))
//			{
//				memcpy(error_temp,response+8,2);
//			}
//			else
//				memcpy(error_temp,CRC_ERROR,2);
//		}
//	}
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp);	
//	return Handle;
//}

//unsigned int CmxADGetCjCChannel(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int *CjCChannel, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_CJC_CHANNEL;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*CjCChannel = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);	
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*CjCChannel = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADSetRTDExcCurrent(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int ExcCurrent, unsigned int *Error)
//{	
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_SET_CJC_CHANNEL;
//	TX_COMMAND[2] = (unsigned char)ExcCurrent;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}   
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADGetRTDExcCurrent(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int *ExcCurrent, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_RTD_EXC;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*ExcCurrent = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);	
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*ExcCurrent = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADSetTemperatureUnit(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int TemperatureUnit, unsigned int *Error)
//{	
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_SET_TEMPERATURE_UNIT;
//	TX_COMMAND[2] = (unsigned char)TemperatureUnit;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}   
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADGetTemperatureUnit(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int *TemperatureUnit, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_TEMPERATURE_UNIT;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*TemperatureUnit = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);	
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*TemperatureUnit = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADSetTCOC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int OCCurrent, unsigned int *Error)
//{	
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_SET_TC_OC;
//	TX_COMMAND[2] = (unsigned char)OCCurrent;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}   
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADGetTCOC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int *OCCurrent, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_TC_OC;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*OCCurrent = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);	
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*OCCurrent = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADGetChannelFault(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int Channel, unsigned int *ChannelFault, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_CHANNEL_FAULT;
//	TX_COMMAND[2] = (unsigned char)Channel;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*ChannelFault = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);	
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*ChannelFault = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADGetCjCValue(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double *CjC, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	unsigned char voltage_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_CJC_VALUE;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					memcpy(error_temp,response+8,2);
//					memcpy(voltage_bin_array,response+2,4);
//					unsigned int voltage_bin = deserialize_uint32(voltage_bin_array);
//					*CjC = unpack754_32(voltage_bin);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					memcpy(error_temp,response+8,2);
//					memcpy(voltage_bin_array,response+2,4);
//					unsigned int voltage_bin = deserialize_uint32(voltage_bin_array);
//					*CjC = unpack754_32(voltage_bin);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		} 		
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp);
//	return Handle;
//}

//unsigned int CmxADSetFilter(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Filter, unsigned int *Error)
//{	
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_SET_FILTER;
//	TX_COMMAND[2] = (unsigned char)Filter;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		} 
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADGetFilter(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int *Filter, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_FILTER;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					*Filter = (unsigned int)response[2]; 
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADGetADCFault(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Fault, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0;
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_ADC_FAULT;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					unsigned char id_array[4] = { 0x00, 0x00, 0x00, 0x00};
//					memcpy(id_array,response+2,4);
//					unsigned int id_u32 = deserialize_uint32(id_array);
//					*Fault = (unsigned int)id_u32;
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					unsigned char id_array[4] = { 0x00, 0x00, 0x00, 0x00};
//					memcpy(id_array,response+2,4);
//					unsigned int id_u32 = deserialize_uint32(id_array);
//					*Fault = (unsigned int)id_u32;
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);	
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp);
//	return Handle;
//}

//unsigned int CmxADSetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewIpAddress[4], unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_SET_IP;
//	TX_COMMAND[2] = (unsigned char)NewIpAddress[0];
//	TX_COMMAND[3] = (unsigned char)NewIpAddress[1];
//	TX_COMMAND[4] = (unsigned char)NewIpAddress[2];
//	TX_COMMAND[5] = (unsigned char)NewIpAddress[3];
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 	
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp);	
//	return Handle;
//}

//unsigned int CmxADGetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int IpAddressReturn[], unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_IP;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 	
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					memcpy(error_temp,response+8,2);
//					unsigned char ip_array[4] = { 0x00, 0x00, 0x00, 0x00 };
//					memcpy(ip_array,response+2,4);
//					IpAddressReturn[0] = (unsigned int)ip_array[0];
//					IpAddressReturn[1] = (unsigned int)ip_array[1]; 
//					IpAddressReturn[2] = (unsigned int)ip_array[2]; 
//					IpAddressReturn[3] = (unsigned int)ip_array[3]; 
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					memcpy(error_temp,response+8,2);
//					unsigned char ip_array[4] = { 0x00, 0x00, 0x00, 0x00 };
//					memcpy(ip_array,response+2,4);
//					IpAddressReturn[0] = (unsigned int)ip_array[0];
//					IpAddressReturn[1] = (unsigned int)ip_array[1]; 
//					IpAddressReturn[2] = (unsigned int)ip_array[2]; 
//					IpAddressReturn[3] = (unsigned int)ip_array[3]; 
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}	
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp);	
//	return Handle;
//}

//unsigned int CmxADResetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_RESET_IP;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 	
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp);  
//	return Handle;
//}

//unsigned int CmxADBoardReset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_BOARD_RESET;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 	
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);	
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp);  	
//	return Handle;
//}

//unsigned int CmxADFirmwareVersion(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double *FWVersion, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_FIRMWARE_VERSION;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					unsigned char fw_array[4] = { 0x00, 0x00, 0x00, 0x00 };
//					memcpy(error_temp,response+8,2);
//					memcpy(fw_array,response+2,4);
//					unsigned int fw_ = deserialize_uint32(fw_array);
//					*FWVersion = unpack754_32(fw_);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);	
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					unsigned char fw_array[4] = { 0x00, 0x00, 0x00, 0x00 };
//					memcpy(error_temp,response+8,2);
//					memcpy(fw_array,response+2,4);
//					unsigned int fw_ = deserialize_uint32(fw_array);
//					*FWVersion = unpack754_32(fw_);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}	
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 	
//	return Handle;
//}

//unsigned int CmxADBoardSerialNumber(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *SerialNo, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0;	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	unsigned char serial_array[4] = { 0x00, 0x00, 0x00, 0x00 };
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_SERIAL_NUMBER;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					memcpy(error_temp,response+8,2);
//					memcpy(serial_array,response+2,4);
//					unsigned int serial_ = deserialize_uint32(serial_array);
//					*SerialNo = serial_;
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					memcpy(error_temp,response+8,2);
//					memcpy(serial_array,response+2,4);
//					unsigned int serial_ = deserialize_uint32(serial_array);
//					*SerialNo = serial_;
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 	
//	return Handle;
//}

//unsigned int CmxADSetBoardID(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewBoardID, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0;	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_SET_BOARD_ID;
//	TX_COMMAND[2] = (unsigned char)NewBoardID;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 		
//	return Handle;
//}

//unsigned int CmxADGetBoardID(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *ReturnBoardID, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0;
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_BOARD_ID;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);	
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (1)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					unsigned char id_array[4] = { 0x00, 0x00, 0x00, 0x00};
//					memcpy(id_array,response+2,4);
//					unsigned int id_u32 = deserialize_uint32(id_array);
//					*ReturnBoardID = (unsigned int)id_u32;
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (1)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					unsigned char id_array[4] = { 0x00, 0x00, 0x00, 0x00};
//					memcpy(id_array,response+2,4);
//					unsigned int id_u32 = deserialize_uint32(id_array);
//					*ReturnBoardID = (unsigned int)id_u32;
//					memcpy(error_temp,response+8,2);
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp);
//	return Handle;
//}

//unsigned int CmxADSetMAC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewMAC[6], unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_SET_MAC;
//	TX_COMMAND[2] = (unsigned char)NewMAC[0];
//	TX_COMMAND[3] = (unsigned char)NewMAC[1];
//	TX_COMMAND[4] = (unsigned char)NewMAC[2];
//	TX_COMMAND[5] = (unsigned char)NewMAC[3];
//	TX_COMMAND[6] = (unsigned char)NewMAC[4];
//	TX_COMMAND[7] = (unsigned char)NewMAC[5];
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 	
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}	
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);	
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 	
//	return Handle;
//}

//unsigned int CmxADGetMAC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int ReturnMac[6], unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_GET_MAC;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 	
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					memcpy(error_temp,response+8,2);
//					unsigned char mac_array[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//					memcpy(mac_array,response+2,6);
//					ReturnMac[0] = (unsigned int)mac_array[0];
//					ReturnMac[1] = (unsigned int)mac_array[1]; 
//					ReturnMac[2] = (unsigned int)mac_array[2]; 
//					ReturnMac[3] = (unsigned int)mac_array[3]; 
//					ReturnMac[4] = (unsigned int)mac_array[4];
//					ReturnMac[5] = (unsigned int)mac_array[5];
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//				{
//					memcpy(error_temp,response+8,2);
//					unsigned char mac_array[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//					memcpy(mac_array,response+2,6);
//					ReturnMac[0] = (unsigned int)mac_array[0];
//					ReturnMac[1] = (unsigned int)mac_array[1]; 
//					ReturnMac[2] = (unsigned int)mac_array[2]; 
//					ReturnMac[3] = (unsigned int)mac_array[3]; 
//					ReturnMac[4] = (unsigned int)mac_array[4];
//					ReturnMac[5] = (unsigned int)mac_array[5];
//				}
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}		
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 
//	return Handle;
//}

//unsigned int CmxADSetSerial(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int SerialNo, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_SET_SERIAL;
//	unsigned char serial_array[4] = {0x00, 0x00, 0x00, 0x00};
//	deserialize_uint32B(serial_array,&SerialNo);
//	TX_COMMAND[2] = (unsigned char)serial_array[0];
//	TX_COMMAND[3] = (unsigned char)serial_array[1];
//	TX_COMMAND[4] = (unsigned char)serial_array[2];
//	TX_COMMAND[5] = (unsigned char)serial_array[3];
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,4); 	
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}	
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 	
//	return Handle;
//}

//unsigned int CmxADSetFirmware(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double Firmware, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_SET_FW;
//    TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	unsigned int fw_bin = pack754_32(Firmware);
//	unsigned char fw_array[4] = {0x00, 0x00, 0x00, 0x00} ;
//	deserialize_uint32B(fw_array,&fw_bin);
//	memcpy(TX_COMMAND+2,fw_array,4);
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, TIMEOUT, 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		} 		
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp);  
//	return Handle;
//}

//unsigned int CmxADRunSelfCalibration(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error)
//{
//	char ip_octet_1[4];
//	char ip_octet_2[4];
//	char ip_octet_3[4];
//	char ip_octet_4[4];
//	char IP_ALL_OCTETS[30];
//	unsigned char buffer[1] ;
//	unsigned char TX_COMMAND[14];
//	unsigned char   *response = NULL;
// 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
//	int udp_error = 0;
//	int serial_error = 0; 	
//	int RX_SIZE = 14;
//	unsigned char crc_temp[10];
//	unsigned int crc_u32=0;
//	TX_COMMAND[0] = (unsigned char)BoardID;
//	TX_COMMAND[1] = AD_RUN_SELF_CALIBRATION;
//	TX_COMMAND[2] = 0x00;
//	TX_COMMAND[3] = 0x00;
//	TX_COMMAND[4] = 0x00;
//	TX_COMMAND[5] = 0x00;
//	TX_COMMAND[6] = 0x00;
//	TX_COMMAND[7] = 0x00;
//	TX_COMMAND[8] = 0x00;
//	TX_COMMAND[9] = 0x00;
//	crcInit();
//	memcpy(crc_temp,TX_COMMAND,10);
//	crc_u32 = CRCCALC(crc_temp,10);
//	unsigned char crc_array[4];
//	deserialize_uint32B(crc_array,&crc_u32);
//	memcpy(TX_COMMAND + 10, crc_array,4);
//	memset(IP_ALL_OCTETS,0,sizeof(buffer));  
// 	itoa(IpAddress[0], ip_octet_1);
//  	itoa(IpAddress[1], ip_octet_2); 
//  	itoa(IpAddress[2], ip_octet_3); 
//  	itoa(IpAddress[3], ip_octet_4); 
//    strcat (IP_ALL_OCTETS, ip_octet_1); 
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_2);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_3);
//	strcat (IP_ALL_OCTETS, ".");
//	strcat (IP_ALL_OCTETS, ip_octet_4);
//	response = malloc(RX_SIZE); 
//	if (Interface == ETHERNET)
//	{
//		unsigned int localdestport = 0;
//		localdestport = DestPort(BoardID);
//		UDPWrite (Handle, localdestport, IP_ALL_OCTETS, TX_COMMAND, 14);
//		udp_error = UDPRead(Handle, response, RX_SIZE, (TIMEOUT*10), 0, 0);
//		if (udp_error == (-6808))
//			memcpy(error_temp,TIMEOUT_ERR,2); 
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
// 		}
//	}
//	else if ((Interface == USB) || (Interface == SERIAL_485))
// 	{
//		SetComTime (Handle, 20.0);
//		ComWrt (Handle, TX_COMMAND, 14);
//		serial_error = ComRd (Handle, response, RX_SIZE);
//		if (serial_error == kRS_IOTimeOut)
//			memcpy(error_temp,TIMEOUT_ERR,2);  
//		else
//		{
//			if (response[0] == BoardID)
//			{
//				memcpy(crc_temp,response,10);
//				memcpy(crc_array,response+10,4);
//				unsigned int crc_result = CRCCALC(crc_temp,10);
//				if (crc_result == deserialize_uint32(crc_array))
//					memcpy(error_temp,response+8,2);
//				else
//					memcpy(error_temp,CRC_ERROR,2);
//			}
//			else
//				memcpy(error_temp,BOARD_ID_ERROR,2);
//		}
//		SetComTime (Handle, 0.025);	
//	}
//	else
//		memcpy(error_temp,INTERFACE_ERROR,2);
//	free(response);
//	*Error = (unsigned int)deserialize_uint32(error_temp); 	
//	return Handle;
//}

unsigned int CmxADSetGain(unsigned int Handle, unsigned int Channel, unsigned int Range, double Gain, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_SET_GAIN;
	TX_COMMAND[2] = (unsigned char) Channel;
	TX_COMMAND[3] = (unsigned char) Range;
	unsigned char level_array[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned int level_u32 = pack754_32(Gain);
	deserialize_uint32B(level_array,&level_u32);
	TX_COMMAND[4] = level_array[0];
	TX_COMMAND[5] = level_array[1];
	TX_COMMAND[6] = level_array[2];
	TX_COMMAND[7] = level_array[3];
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	 
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADGetGain (unsigned int Handle, unsigned int Channel, unsigned int Range, double *Gain, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char scale_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_GET_GAIN;
	TX_COMMAND[2] = (unsigned char) Channel;
	TX_COMMAND[3] = (unsigned char) Range;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(scale_bin_array,response+2,4);
				unsigned int gain_bin = deserialize_uint32(scale_bin_array);
				*Gain = unpack754_32(gain_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADSetOffset(unsigned int Handle, unsigned int Channel, unsigned int Range, double Offset, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_SET_OFFSET;
	TX_COMMAND[2] = (unsigned char) Channel;
	TX_COMMAND[3] = (unsigned char) Range;
	unsigned char level_array[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned int level_u32 = pack754_32(Offset);
	deserialize_uint32B(level_array,&level_u32);
	TX_COMMAND[4] = level_array[0];
	TX_COMMAND[5] = level_array[1];
	TX_COMMAND[6] = level_array[2];
	TX_COMMAND[7] = level_array[3];
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxADGetOffset (unsigned int Handle, unsigned int Channel, unsigned int Range, double *Offset, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char scale_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = AD_GET_OFFSET;
	TX_COMMAND[2] = (unsigned char) Channel;
	TX_COMMAND[3] = (unsigned char) Range;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(scale_bin_array,response+2,4);
				unsigned int gain_bin = deserialize_uint32(scale_bin_array);
				*Offset = unpack754_32(gain_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

/*	Analog Input Functions END	*/

/*	Analog Output Functions BEGIN	*/
unsigned int CmxDAOpenBoard (char* IPAddress, unsigned int BoardID, unsigned int BoardReset, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14]={0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned int Handle = 0;	
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_OPEN_BOARD;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	int tcp_error = 0;
	unsigned int port = DestPort(BoardID);
	tcp_error = ConnectToTCPServerEx (&Handle, port, IPAddress, 0, 0, 2000, TCP_ANY_LOCAL_PORT);
	if (tcp_error < 0)
		memcpy(error_temp, NO_CONNECTION_ERROR, 2); 
	else
	{	
		if (BoardReset == 1)
		{
			int messageSize = 14;
			int bytesToWrite = 14;
			int bytesWritten = 0;
			while (bytesToWrite > 0)
			{
				bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
				if (bytesWritten>=0)
					bytesToWrite -= bytesWritten;
				else
					break;
			}
			if (bytesWritten < 0)
				memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
			else
			{
				unsigned int bytestoread = 14;
				int byteread = 0;
				messageSize = 14;
				int totalbytes = 0;
				while (totalbytes < 13)
				{
					byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
					if (byteread > 0)
					{
						bytestoread -= byteread;
						totalbytes = byteread + totalbytes;
					}
					else if (byteread < 0)
						break;
				}
				if ((byteread < 0) || (totalbytes < 14))
					memcpy(error_temp,TIMEOUT_ERR,2); 
				else
				{
					memcpy(crc_temp,response,10);
					memcpy(crc_array,response+10,4);
					unsigned int crc_result = CRCCALC(crc_temp,10);
					if (crc_result == deserialize_uint32(crc_array))
						memcpy(error_temp,response+8,2);
					else
						memcpy(error_temp,CRC_ERROR,2);
				}
			}	
		}
		else
		{
			error_temp[0] = 0x01;
			error_temp[1] = 0x00;
		}	
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	Sleep(500);
	return Handle;
}

unsigned int CmxDASetReadBack (unsigned int Handle, unsigned int ReadBackOnOff, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_READBACK;
	TX_COMMAND[2] = (unsigned char)ReadBackOnOff;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDAGetReadBack (unsigned int Handle, unsigned int *ReadBack, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_READBACK;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);	
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*ReadBack = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDASetChannelMode (unsigned int Handle, unsigned int Channel, unsigned int Mode, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_CHANNEL_MODE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char)Mode;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
		
	}
	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDAGetChannelMode (unsigned int Handle, unsigned int Channel, unsigned int *ChannelMode, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_CHANNEL_MODE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*ChannelMode = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDASetRange (unsigned int Handle, unsigned int Channel, unsigned int Range, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_RANGE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char)Range;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDAGetRange (unsigned int Handle, unsigned int Channel, unsigned int *Range, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_RANGE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*Range = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDASetUpdateMode (unsigned int Handle, unsigned int UpdateMode, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_UPDATE_MODE;
	TX_COMMAND[2] = (unsigned char)UpdateMode;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	 
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDAGetUpdateMode (unsigned int Handle, unsigned int *UpdateMode, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 	

	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_UPDATE_MODE;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*UpdateMode = (unsigned int)response[2];
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDAUpdateOutput (unsigned int Handle, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_UPDATE_OUTPUT;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDASetBroadcast (unsigned int Handle, unsigned int BroadcastOnOff, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_BROADCAST;
	TX_COMMAND[2] = (unsigned char)BroadcastOnOff;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDAGetBroadcast (unsigned int Handle, unsigned int *Broadcast, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_BROADCAST;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
 
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*Broadcast = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDASetBroadcastData (unsigned int Handle, double BroadcastData, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_BROADCAST_DATA;
	unsigned int broadcast_data_u32 = 0;
	broadcast_data_u32 = pack754_32(BroadcastData);
	unsigned char temp[4] = {0, 0, 0, 0};
	deserialize_uint32B(temp,&broadcast_data_u32);
	TX_COMMAND[2] = temp[0];
	TX_COMMAND[3] = temp[1];
	TX_COMMAND[4] = temp[2];
	TX_COMMAND[5] = temp[3];
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDAGetBroadcastData (unsigned int Handle, double *BroadcastData, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_BROADCAST_DATA;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	unsigned char voltage_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(voltage_bin_array,response+2,4);
				unsigned int voltage_bin = deserialize_uint32(voltage_bin_array);
				*BroadcastData = unpack754_32(voltage_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDASetGain (unsigned int Handle, unsigned int Channel, double Gain, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_GAIN;
	TX_COMMAND[2] = (unsigned char) Channel;
	unsigned char level_array[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned int level_u32 = pack754_32(Gain);
	deserialize_uint32B(level_array,&level_u32);
	TX_COMMAND[3] = level_array[0];
	TX_COMMAND[4] = level_array[1];
	TX_COMMAND[5] = level_array[2];
	TX_COMMAND[6] = level_array[3];
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDAGetGain (unsigned int Handle, unsigned int Channel, double *Gain, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char scale_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_GAIN;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(scale_bin_array,response+2,4);
				unsigned int gain_bin = deserialize_uint32(scale_bin_array);
				*Gain = unpack754_32(gain_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDASetOffset (unsigned int Handle, unsigned int Channel, double Offset, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_OFFSET;
	TX_COMMAND[2] = (unsigned char) Channel;
	unsigned char level_array[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned int level_u32 = pack754_32(Offset);
	deserialize_uint32B(level_array,&level_u32);
	TX_COMMAND[3] = level_array[0];
	TX_COMMAND[4] = level_array[1];
	TX_COMMAND[5] = level_array[2];
	TX_COMMAND[6] = level_array[3];
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDAGetOffset (unsigned int Handle, unsigned int Channel, double *Offset, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char scale_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_OFFSET;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(scale_bin_array,response+2,4);
				unsigned int gain_bin = deserialize_uint32(scale_bin_array);
				*Offset = unpack754_32(gain_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDAStart (unsigned int Handle, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_START;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDAStop (unsigned int Handle, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};; 	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_STOP;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDASetChannelData (unsigned int Handle, unsigned int Channel, double OutputVoltage, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_CHANNEL_DATA;
	TX_COMMAND[2] = (unsigned char)Channel;
	unsigned int output_bin = 0;
	unsigned char output_bin_array[4] = {0,0,0,0};
	output_bin = pack754_32(OutputVoltage);
	deserialize_uint32B(output_bin_array,&output_bin);
	TX_COMMAND[3] = (unsigned char) output_bin_array[0];
	TX_COMMAND[4] = (unsigned char) output_bin_array[1];
	TX_COMMAND[5] = (unsigned char) output_bin_array[2];
	TX_COMMAND[6] = (unsigned char) output_bin_array[3];
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDAGetChannelData (unsigned int Handle, unsigned int Channel, double *OutputVoltage, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_CHANNEL_DATA;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4); 
	unsigned char voltage_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(voltage_bin_array,response+2,4);
				unsigned int voltage_bin = deserialize_uint32(voltage_bin_array);
				*OutputVoltage = unpack754_32(voltage_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDASetIP (unsigned int Handle, unsigned int NewIpAddress[4], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00}; 	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_IP;
	TX_COMMAND[2] = (unsigned char)NewIpAddress[0];
	TX_COMMAND[3] = (unsigned char)NewIpAddress[1];
	TX_COMMAND[4] = (unsigned char)NewIpAddress[2];
	TX_COMMAND[5] = (unsigned char)NewIpAddress[3];
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDAGetIP (unsigned int Handle, unsigned int IpAddressReturn[], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_IP;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				unsigned char ip_array[4] = { 0x00, 0x00, 0x00, 0x00 };
				memcpy(ip_array,response+2,4);
				IpAddressReturn[0] = (unsigned int)ip_array[0];
				IpAddressReturn[1] = (unsigned int)ip_array[1]; 
				IpAddressReturn[2] = (unsigned int)ip_array[2]; 
				IpAddressReturn[3] = (unsigned int)ip_array[3]; 
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);	
	return Handle;
}

unsigned int CmxDAResetIP (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_RESET_IP;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDABoardReset (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_BOARD_RESET;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);  	
	return Handle;
}

unsigned int CmxDAFirmwareVersion (unsigned int Handle, double *FWVersion, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_FIRMWARE_VERSION;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				unsigned char fw_array[4] = { 0x00, 0x00, 0x00, 0x00 };
				memcpy(error_temp,response+8,2);
				memcpy(fw_array,response+2,4);
				unsigned int fw_ = deserialize_uint32(fw_array);
				*FWVersion = unpack754_32(fw_);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 	
	return Handle;
}

unsigned int CmxDABoardSerialNumber (unsigned int Handle, unsigned int *SerialNo, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SERIAL_NUMBER;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	unsigned char serial_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(serial_array,response+2,4);
				unsigned int serial_ = deserialize_uint32(serial_array);
				*SerialNo = serial_;
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 	
	return Handle;
}

unsigned int CmxDASetBoardID (unsigned int Handle, unsigned int NewBoardID, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_BOARD_ID;
	TX_COMMAND[2] = (unsigned char)NewBoardID;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDAGetBoardID (unsigned int Handle, unsigned int *ReturnBoardID, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_BOARD_ID;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);;
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				unsigned char id_array[4] = { 0x00, 0x00, 0x00, 0x00};
				memcpy(id_array,response+2,4);
				unsigned int id_u32 = deserialize_uint32(id_array);
				*ReturnBoardID = (unsigned int)id_u32;
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxDASetMAC (unsigned int Handle, unsigned int NewMAC[6], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_MAC;
	TX_COMMAND[2] = (unsigned char)NewMAC[0];
	TX_COMMAND[3] = (unsigned char)NewMAC[1];
	TX_COMMAND[4] = (unsigned char)NewMAC[2];
	TX_COMMAND[5] = (unsigned char)NewMAC[3];
	TX_COMMAND[6] = (unsigned char)NewMAC[4];
	TX_COMMAND[7] = (unsigned char)NewMAC[5];
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
 
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDAGetMAC (unsigned int Handle, unsigned int ReturnMac[], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_GET_MAC;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				unsigned char mac_array[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
				memcpy(mac_array,response+2,6);
				ReturnMac[0] = (unsigned int)mac_array[0];
				ReturnMac[1] = (unsigned int)mac_array[1]; 
				ReturnMac[2] = (unsigned int)mac_array[2]; 
				ReturnMac[3] = (unsigned int)mac_array[3]; 
				ReturnMac[4] = (unsigned int)mac_array[4];
				ReturnMac[5] = (unsigned int)mac_array[5];
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 	
	return Handle;
}

unsigned int CmxDASetSerial (unsigned int Handle, unsigned int SerialNo, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_SERIAL;
	unsigned char serial_array[4] = {0x00, 0x00, 0x00, 0x00};
	deserialize_uint32B(serial_array,&SerialNo);
	TX_COMMAND[2] = (unsigned char)serial_array[0];
	TX_COMMAND[3] = (unsigned char)serial_array[1];
	TX_COMMAND[4] = (unsigned char)serial_array[2];
	TX_COMMAND[5] = (unsigned char)serial_array[3];
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 	
	return Handle;
}

unsigned int CmxDASetFirmware (unsigned int Handle, double Firmware, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_FW;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	unsigned int fw_bin = pack754_32(Firmware);
	unsigned char fw_array[4] = {0x00, 0x00, 0x00, 0x00} ;
	deserialize_uint32B(fw_array,&fw_bin);
	memcpy(TX_COMMAND+2,fw_array,4);
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);   
	return Handle;
}

unsigned int CmxDADiscardBoard (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_DISCARD_BOARD;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		error_temp[0] = 1;
		error_temp[1] = 0;
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	DisconnectFromTCPServer (Handle);
	return 0;
}

unsigned int CmxDASetGenMode (unsigned int Handle, unsigned int Channel, unsigned int GenerationMode, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_GENERATION_MODE;
	TX_COMMAND[2] = (unsigned char) Channel;
	TX_COMMAND[3] = (unsigned char) GenerationMode;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
 	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDASetBufferSize (unsigned int Handle, unsigned int Channel, unsigned int BufferSize, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_BUFFER_SIZE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char)BufferSize;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDAWriteArrayPoint (unsigned int Handle, unsigned int Channel, unsigned int ArrayIndex, double Data, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_WRITE_ARRAY_POINT;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char) ArrayIndex;
	unsigned int output_bin = 0;
	unsigned char output_bin_array[4] = {0,0,0,0};
	output_bin = pack754_32(Data);
	deserialize_uint32B(output_bin_array,&output_bin);
	TX_COMMAND[4] = (unsigned char) output_bin_array[0];
	TX_COMMAND[5] = (unsigned char) output_bin_array[1];
	TX_COMMAND[6] = (unsigned char) output_bin_array[2];
	TX_COMMAND[7] = (unsigned char) output_bin_array[3];
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxDASetUpdateTime (unsigned int Handle, unsigned int TimeinMilliSecond, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = DA_SET_UPDATE_TIME;
	TX_COMMAND[2] = (unsigned char) TimeinMilliSecond;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}
/****************************************************************1553 Functions**********************************************************/
unsigned int Cmx1553OpenBoard (char* IPAddress, unsigned int BoardID, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	unsigned int Handle = 0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_OPEN_BOARD;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	unsigned int port = DestPort(BoardID);
	tcp_error = ConnectToTCPServerEx (&Handle, port, IPAddress, 0, 0, TIMEOUT, TCP_ANY_LOCAL_PORT);
	if (tcp_error < 0)
		memcpy(error_temp, NO_CONNECTION_ERROR, 2);
	else
	{
		tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
		if (tcp_error < 0)
			memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
		else
		{
			unsigned int bytestoread = 80;
			int byteread = 0;
			unsigned int totalbyte = 0;
			unsigned int messagesize = 80;
			clock_t start_time = clock();
			while (clock() < start_time + TIMEOUT)
			{
				byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
				waitMs();
				if (byteread >=0)
				{
					bytestoread -= byteread;
					totalbyte = byteread + totalbyte;
				}
				if (totalbyte == messagesize)
				break;
			}
			if (totalbyte <= 0)
				memcpy(error_temp,TIMEOUT_ERR,2);
			else
			{
				memcpy(crc_temp,response,76);
				memcpy(crc_array,response+76,4);
				unsigned int crc_result = CRCCALC(crc_temp,76);
				if (crc_result == deserialize_uint32(crc_array))
					memcpy(error_temp,response+74,2);
				else
					memcpy(error_temp,CRC_ERROR,2);
			}
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	free(response);
	return Handle;
}

unsigned int Cmx1553InitBoard (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_INIT_BOARD;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553ResetBoard (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RESET_BOARD;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BoardSetMode (unsigned int Handle, unsigned int BCMode, unsigned int RT1Mode, unsigned int RT2Mode, unsigned int MTMode, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BOARD_SET_1553_MODE;
	TX_COMMAND[2] = (unsigned char)BCMode;
	TX_COMMAND[3] = (unsigned char)RT1Mode;
	TX_COMMAND[4] = (unsigned char)RT2Mode;
	TX_COMMAND[5] = (unsigned char)MTMode;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BoardGetMode(unsigned int Handle, unsigned int *Mode, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BOARD_GET_1553_MODE;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+74,2);
				*Mode = (unsigned int)response[2];
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553ResetIC (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_2130_RESET;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553SharedInit (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_SHARED_INIT;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcInitMsgData (unsigned int Handle, unsigned int *Data, unsigned int DataCount, unsigned int MessageType, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
 	unsigned char packet_temp[4]={0x00,0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_INIT_MSG_DATA;
	if(MessageType==0 || MessageType==1||MessageType==3 ){
		for(int i=0;i<32;i+=1){
			deserialize_uint32B(packet_temp,&Data[i]);
			memcpy(TX_COMMAND + (2+(i*2)), packet_temp,2);
		}
	}
	TX_COMMAND[66] = (unsigned char)DataCount;
	TX_COMMAND[67] = (unsigned char)MessageType;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcInitMsg (unsigned int Handle, unsigned int Bus, unsigned int MessageType, unsigned int RtAddr, unsigned int RtSubAddr, unsigned int Mcode, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_INIT_MSG;
	TX_COMMAND[2] = (unsigned char)Bus;
	TX_COMMAND[3] = (unsigned char)MessageType;
	TX_COMMAND[4] = (unsigned char)RtAddr;
	TX_COMMAND[5] = (unsigned char)RtSubAddr;
	TX_COMMAND[6] = (unsigned char)Mcode;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcInitMsgList (unsigned int Handle, unsigned int loop, unsigned int MsgGapTime, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	unsigned char packet_temp[4]={0,0,0,0};
	deserialize_uint32B(packet_temp, &MsgGapTime);
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_INIT_MSG_INST_LIST;
	TX_COMMAND[2] = (unsigned char)loop;
	TX_COMMAND[3] = (unsigned char)packet_temp[0];
	TX_COMMAND[4] = (unsigned char)packet_temp[1];
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcSetTTAG (unsigned int Handle,unsigned int HiLo, unsigned int BcTTAG, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_SET_TTAG;
	TX_COMMAND[2] = (unsigned char)HiLo;
	TX_COMMAND[3] = (unsigned char)BcTTAG;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcGetMsgInProgress (unsigned int Handle, unsigned int *MsgProgress, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_GET_PROGRESS;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*MsgProgress = (unsigned int)response[2];
				memcpy(error_temp,response+74,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcGetMsgBlock(unsigned int Handle, unsigned int *msgblock, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	unsigned char packet_temp[2]={0,0};
    for (int i = 0; i < 32; i++) {
        msgblock[i] = 0;
    }
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_GET_MSG_BLOCK;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
			{
				for (int i = 2; i < 21; i += 2) 
				{
					packet_temp[0] = response[i];
					packet_temp[1] = response[i + 1];
					msgblock[(i / 2) - 1] = deserialize_uint16(packet_temp);
				}
				memcpy(error_temp,response+74,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcGetMsgData (unsigned int Handle, unsigned int *msgdata, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	uint8_t packet_temp[2]={0,0};
    for (int i = 0; i < 32; i++) {
        msgdata[i] = 0;
    }
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_GET_MSG_DATA;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
			{
				for (int i = 2; i < 66; i += 2) {
					packet_temp[0] = response[i];
					packet_temp[1] = response[i + 1];
					msgdata[(i / 2) - 1] = deserialize_uint16(packet_temp);
				}
				memcpy(error_temp,response+74,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcEnable (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_ENABLE;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcStop (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_STOP;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcTrigger (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_TRIGGER;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553BcStart (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_BC_START;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553MtSetTTAG (unsigned int Handle, unsigned int HiLo, unsigned int MTTag, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_MT_SET_TTAG;
	TX_COMMAND[2] = (unsigned char)HiLo;
	TX_COMMAND[3] = (unsigned char)MTTag;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553MtSetFilter (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_MT_SET_FILTER;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553MtGetLastCommand (unsigned int Handle, unsigned int *LastCommand ,unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	unsigned char packet_temp[4]={0,0,0,0};
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_MT_GET_LAST_COMMAND;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
			{
				packet_temp[0] = response[2];
				packet_temp[1] = response[3];
				*LastCommand = deserialize_uint32(packet_temp);
				memcpy(error_temp,response+74,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553MtGetLastMessage (unsigned int Handle, unsigned int *LastMessage, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	unsigned char packet_temp[4]={0,0,0,0};
    for (int i = 0; i < 32; i++) {
    	LastMessage[i] = 0;
    }
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_MT_GET_LAST_MSG;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
			{
				for (int i = 2; i < 66; i += 2) {
					packet_temp[0] = response[i];
					packet_temp[1] = response[i + 1];
					LastMessage[(i / 2) - 1] = deserialize_uint32(packet_temp);
				}
				memcpy(error_temp,response+74,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553MtEnable (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_MT_ENABLE;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553MtStop(unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_MT_STOP;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt1Init (unsigned int Handle, unsigned int SupportBroadcast, unsigned int UndefinedMcodesValid, unsigned int UseSMCP, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT1_INIT;
	TX_COMMAND[2] = (unsigned char)SupportBroadcast;
	TX_COMMAND[3] = (unsigned char)UndefinedMcodesValid;
	TX_COMMAND[4] = (unsigned char)UseSMCP;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt1Init1 (unsigned int Handle, unsigned int UseSMCP, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT1_INIT1;
	TX_COMMAND[2] = (unsigned char)UseSMCP;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt1Init2 (unsigned int Handle, unsigned int illegalcmd_detect,unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT1_INIT2;
	TX_COMMAND[2] = (unsigned char)illegalcmd_detect;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt1SetAddr (unsigned int Handle,unsigned int addr, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT1_SET_ADDR;
	TX_COMMAND[2] = (unsigned char)addr;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt1SetTTAG (unsigned int Handle,unsigned int RT1ttag ,unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT1_SET_TTAG;
	TX_COMMAND[2] = (unsigned char)RT1ttag;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt1SetTXData (unsigned int Handle, unsigned int *rt1txdata, unsigned int data_count, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
 	unsigned char packet_temp[4]={0x00,0x00,0,0};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT1_SET_TX_DATA;
	for(int i=0;i<32;i+=1){
		deserialize_uint32B(packet_temp,&rt1txdata[i]);
		memcpy(TX_COMMAND + (2+(i*2)), packet_temp,2);
	}
	TX_COMMAND[66] = (unsigned char)data_count;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt1Enable (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT1_ENABLE;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt1Stop (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT1_STOP;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt1Start (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT1_START;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt2Init (unsigned int Handle, unsigned int support_broadcast,unsigned int undef_mcodes_valid,unsigned int use_smcp, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT2_INIT;
	TX_COMMAND[2] = (unsigned char)support_broadcast;
	TX_COMMAND[3] = (unsigned char)undef_mcodes_valid;
	TX_COMMAND[4] = (unsigned char)use_smcp;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt2Init1 (unsigned int Handle, unsigned int use_smcp, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT2_INIT1;
	TX_COMMAND[2] = (unsigned char)use_smcp;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt2Init2 (unsigned int Handle, unsigned int illegalcmd_detect,unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT2_INIT2;
	TX_COMMAND[2] = (unsigned char)illegalcmd_detect;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt2SetAddr (unsigned int Handle,unsigned int addr, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT2_SET_ADDR;
	TX_COMMAND[2] = (unsigned char)addr;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			Sleep(1);
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt2SetTTAG (unsigned int Handle, unsigned int RT1ttag ,unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT2_SET_TTAG;
	TX_COMMAND[2] = (unsigned char)RT1ttag;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt2SetTXData (unsigned int Handle,unsigned int *rt2txdata, unsigned int data_count, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
 	unsigned char packet_temp[2]={0x00,0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT2_SET_TX_DATA;
	for(int i=0;i<32;i+=1){
		deserialize_uint32B(packet_temp, &rt2txdata[i]);
		memcpy(TX_COMMAND+2+(i*2), packet_temp,2);
	}
	TX_COMMAND[66] = (unsigned char)data_count;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt2Enable (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT2_ENABLE;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt2Stop (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT2_STOP;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553Rt2Start (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] = {0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT2_START;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553RtGetMessage (unsigned int Handle, unsigned int *RTmessage,unsigned int rt_num, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	unsigned char packet_temp[2]={0,0};
    for (int i = 0; i < 32; i++) {
    	RTmessage[i] = 0;
    }
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT_GET_MSG;
	TX_COMMAND[2] = (unsigned char)rt_num;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy (error_temp, response + 74, 2);
		    	packet_temp[0]=response[2];
		    	packet_temp[1]=response[3];
		    	RTmessage[0]=deserialize_uint16(packet_temp);
				printf("RT%d Message Command word=%x\n",rt_num,RTmessage[0]);
		    	packet_temp[0]=response[4];
		    	packet_temp[1]=response[5];
		    	RTmessage[1]=deserialize_uint16(packet_temp);
				printf("RT%d Message Data Length=%x\n",rt_num,RTmessage[1]);
		    	packet_temp[0]=response[6];
		    	packet_temp[1]=response[7];
		    	RTmessage[1]=deserialize_uint16(packet_temp);
				printf("RT%d Mode Code=%x\n",rt_num,RTmessage[2]);
			    for (int i = 8; i < 72; i += 2) {
					packet_temp[0] = response[i];
					packet_temp[1] = response[i + 1];
					RTmessage[(i / 2) - 1] = deserialize_uint16(packet_temp);
			    }
				 for(int i =0;i<32;i++){
	    			printf("RT%d message[%d]=%x\n",rt_num,i,RTmessage[i+3]);
	   			 }
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553RtCheckAddr (unsigned int Handle, unsigned int rt_num, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_RT_CHECK_ADDR;
	TX_COMMAND[2] = (unsigned char)rt_num;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 80;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 80;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2);
		else
		{
			memcpy(crc_temp,response,76);
			memcpy(crc_array,response+76,4);
			unsigned int crc_result = CRCCALC(crc_temp,76);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+74,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int Cmx1553DiscardBoard (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[80] ={0};
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 80;
	unsigned char crc_temp[76];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = CM_DISCARD_BOARD;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,76);
	crc_u32 = CRCCALC(crc_temp,76);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 76, crc_array,4);
	response = malloc(RX_SIZE);
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 80, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	tcp_error = DisconnectFromTCPServer (Handle);
	if (tcp_error < 0)
		memcpy(error_temp,GENERIC_ERROR,2);
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;	
}

/****************************************************************6022 Functions BEGIN****************************************************************/
unsigned int CmxTCOpenBoard (char* IPAddress, unsigned int BoardID, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14]={0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned int Handle = 0;	
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_OPEN_BOARD;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	int tcp_error = 0;
	unsigned int port = DestPort(BoardID);
	tcp_error = ConnectToTCPServerEx (&Handle, port, IPAddress, 0, 0, 2000, TCP_ANY_LOCAL_PORT);
	if (tcp_error < 0)
		memcpy(error_temp, NO_CONNECTION_ERROR, 2); 
	else
	{	
		for (int iteration = 0; iteration < 3; iteration++)
		{
		int messageSize = 14;
		int bytesToWrite = 14;
		int bytesWritten = 0;
		while (bytesToWrite > 0)
		{
			bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
			if (bytesWritten>=0)
				bytesToWrite -= bytesWritten;
			else
				break;
		}
		if (bytesWritten < 0)
			memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
		else
		{
			unsigned int bytestoread = 14;
			int byteread = 0;
			messageSize = 14;
			int totalbytes = 0;
			while (totalbytes < 13)
			{
				byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
				if (byteread > 0)
				{
					bytestoread -= byteread;
					totalbytes = byteread + totalbytes;
				}
				else if (byteread < 0)
					break;
			}
			if ((byteread < 0) || (totalbytes < 14))
				memcpy(error_temp,TIMEOUT_ERR,2); 
			else
			{
				memcpy(crc_temp,response,10);
				memcpy(crc_array,response+10,4);
				unsigned int crc_result = CRCCALC(crc_temp,10);
				if (crc_result == deserialize_uint32(crc_array))
				{
					memcpy(error_temp,response+8,2);
					break;
				}
				else
					memcpy(error_temp,CRC_ERROR,2);
			}
		}	
		}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	Sleep(1000);
	return Handle;
}

unsigned int CmxTCSetType (unsigned int Handle, unsigned int Channel, unsigned int TCType, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_SET_TYPE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = (unsigned char)TCType;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	for (int iteration = 0; iteration < 3; iteration++)
	{
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				break;
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);	
	return Handle;
}

unsigned int CmxTCGetType (unsigned int Handle, unsigned int Channel, unsigned int *TCType, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	int tcp_error = 0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_GET_TYPE;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*TCType = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCSetCjCChannel (unsigned int Handle, unsigned int CjcChannel, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_SET_CJC_CHANNEL;
	TX_COMMAND[2] = (unsigned char)CjcChannel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);	
	return Handle;
}

unsigned int CmxTCGetCjcChannel (unsigned int Handle, unsigned int *CjcChannel, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	int tcp_error = 0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_GET_CJC_CHANNEL;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*CjcChannel = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCSetTemperatureUnit (unsigned int Handle, unsigned int TemperatureUnit, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char  response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_SET_TEMPERATURE_UNIT;
	TX_COMMAND[2] = (unsigned char)TemperatureUnit;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (Handle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (Handle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);	
	return Handle;
}

unsigned int CmxTCGetTemperatureUnit (unsigned int Handle, unsigned int *TemperatureUnit, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	int tcp_error = 0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_GET_TEMPERATURE_UNIT;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*TemperatureUnit = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCSetTrigger (unsigned int Handle, unsigned int TriggerType, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_SET_TRIGGER;
	TX_COMMAND[2] = (unsigned char)TriggerType;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxTCGetTrigger (unsigned int Handle, unsigned int *TriggerType, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_GET_TRIGGER;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				*TriggerType = (unsigned int)response[2]; 
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);  
	return Handle;
}

unsigned int CmxTCStartConversion (unsigned int Handle, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_START_CONVERSION;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCStopConversion (unsigned int Handle, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_STOP_CONVERSION;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCReadBuffer (unsigned int Handle, unsigned int TimeOut, unsigned int NumberOfSamples, double *Buffer, unsigned int *FetchedSampleCount, unsigned int *Error)
{
	unsigned int bytestoread = 0;
	int byteread = 0;
	unsigned char temp[4] = {0, 0, 0, 0};
	unsigned int u32 = 0;
	unsigned char *buffer_;
	unsigned int totalbyte = 0;
	unsigned int messagesize = NumberOfSamples;
	unsigned int sample_size = NumberOfSamples;
	messagesize = messagesize * 32;
	sample_size = sample_size * 8;
	bytestoread = messagesize;
	buffer_ = malloc(bytestoread);
	totalbyte = 0;
	bytestoread = messagesize;
	clock_t start_time = clock();
	while (clock() < start_time + TimeOut)
	{
		byteread = ClientTCPRead (Handle, &buffer_[messagesize - bytestoread], bytestoread, 0);
		waitMs();
		if (byteread >=0)
		{
			bytestoread -= byteread;
			totalbyte = byteread + totalbyte;
		}
		if (totalbyte == messagesize)
		break;
	}
	if (totalbyte >0)
	{
		for (int i = 0; i < sample_size; i++)
		{
			memcpy (temp, buffer_ + (i * 4), 4);
			u32 = deserialize_uint32(temp);
			Buffer[i] = unpack754_32(u32);
		}
		*Error = 0x01;
	}
	else
		*Error = 0x04;	
	free(buffer_);
	*FetchedSampleCount = (totalbyte / 4);
	return 0;
}

unsigned int CmxTCSetCjcValue (unsigned int Handle, double CjcTemperatureValue, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_SET_CJC_VALUE;
	unsigned int output_bin = 0;
	unsigned char output_bin_array[4] = {0,0,0,0};
	output_bin = pack754_32(CjcTemperatureValue);
	deserialize_uint32B(output_bin_array,&output_bin);
	TX_COMMAND[2] = (unsigned char) output_bin_array[0];
	TX_COMMAND[3] = (unsigned char) output_bin_array[1];
	TX_COMMAND[4] = (unsigned char) output_bin_array[2];
	TX_COMMAND[5] = (unsigned char) output_bin_array[3];
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCGetCjcValue (unsigned int Handle, double *CjcTemperatureValue, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_GET_CJC_VALUE;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4); 
	unsigned char voltage_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(voltage_bin_array,response+2,4);
				unsigned int voltage_bin = deserialize_uint32(voltage_bin_array);
				*CjcTemperatureValue = unpack754_32(voltage_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCSetGain (unsigned int Handle, unsigned int Channel, double Gain, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_SET_GAIN;
	TX_COMMAND[2] = (unsigned char) Channel;
	unsigned int output_bin = 0;
	unsigned char output_bin_array[4] = {0,0,0,0};
	output_bin = pack754_32(Gain);
	deserialize_uint32B(output_bin_array,&output_bin);
	TX_COMMAND[3] = (unsigned char) output_bin_array[0];
	TX_COMMAND[4] = (unsigned char) output_bin_array[1];
	TX_COMMAND[5] = (unsigned char) output_bin_array[2];
	TX_COMMAND[6] = (unsigned char) output_bin_array[3];
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCGetGain (unsigned int Handle, unsigned int Channel, double *Gain, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char scale_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_GET_GAIN;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(scale_bin_array,response+2,4);
				unsigned int gain_bin = deserialize_uint32(scale_bin_array);
				*Gain = unpack754_32(gain_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	free(response);	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxTCSetOffset (unsigned int Handle, unsigned int Channel, double Offset, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_SET_OFFSET;
	TX_COMMAND[2] = (unsigned char) Channel;
	unsigned int output_bin = 0;
	unsigned char output_bin_array[4] = {0,0,0,0};
	output_bin = pack754_32(Offset);
	deserialize_uint32B(output_bin_array,&output_bin);
	TX_COMMAND[3] = (unsigned char) output_bin_array[0];
	TX_COMMAND[4] = (unsigned char) output_bin_array[1];
	TX_COMMAND[5] = (unsigned char) output_bin_array[2];
	TX_COMMAND[6] = (unsigned char) output_bin_array[3];
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCGetOffset (unsigned int Handle, unsigned int Channel, double *Offset, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned char scale_bin_array[4] = { 0x00, 0x00, 0x00, 0x00 };
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_GET_OFFSET;
	TX_COMMAND[2] = (unsigned char)Channel;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				memcpy(scale_bin_array,response+2,4);
				unsigned int gain_bin = deserialize_uint32(scale_bin_array);
				*Offset = unpack754_32(gain_bin);
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	free(response);	
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return Handle;
}

unsigned int CmxTCSetTemperatureGain (unsigned int Handle, unsigned int Channel, unsigned int Type, double TemperatureGain, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_SET_TEMP_GAIN;
	TX_COMMAND[2] = (unsigned char) Channel;
	TX_COMMAND[3] = (unsigned char) Type;
	unsigned int output_bin = 0;
	unsigned char output_bin_array[4] = {0,0,0,0};
	output_bin = pack754_32(TemperatureGain);
	deserialize_uint32B(output_bin_array,&output_bin);
	TX_COMMAND[4] = (unsigned char) output_bin_array[0];
	TX_COMMAND[5] = (unsigned char) output_bin_array[1];
	TX_COMMAND[6] = (unsigned char) output_bin_array[2];
	TX_COMMAND[7] = (unsigned char) output_bin_array[3];
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCSetTemperatureOffset (unsigned int Handle, unsigned int Channel, unsigned int Type, double TemperatureOffset, unsigned int *Error)
{	
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_SET_TEMP_OFFSET;
	TX_COMMAND[2] = (unsigned char) Channel;
	unsigned int output_bin = 0;
	unsigned char output_bin_array[4] = {0,0,0,0};
	output_bin = pack754_32(TemperatureOffset);
	deserialize_uint32B(output_bin_array,&output_bin);
	TX_COMMAND[3] = (unsigned char) output_bin_array[0];
	TX_COMMAND[4] = (unsigned char) output_bin_array[1];
	TX_COMMAND[5] = (unsigned char) output_bin_array[2];
	TX_COMMAND[6] = (unsigned char) output_bin_array[3];
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	else
	{	
		unsigned int bytestoread = 14;
		int byteread = 0;
		unsigned int totalbyte = 0;
		unsigned int messagesize = 14;
		clock_t start_time = clock();
		while (clock() < start_time + TIMEOUT)
		{
			byteread = ClientTCPRead (Handle, &response[messagesize - bytestoread], bytestoread, 0);
			waitMs();
			if (byteread >=0)
			{
				bytestoread -= byteread;
				totalbyte = byteread + totalbyte;
			}
			if (totalbyte == messagesize)
			break;
		}
		if (totalbyte <= 0)
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return Handle;
}

unsigned int CmxTCDiscardBoard (unsigned int Handle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};; 	
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x01;
	TX_COMMAND[1] = TC_DISCARD_BOARD;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE); 
	int timeout = 100;
	int tcp_error = 0;
	tcp_error = ClientTCPWrite (Handle, TX_COMMAND, 14, timeout);
	if (tcp_error < 0)
		memcpy(error_temp,COMMAND_SEND_ERROR,2);
	DisconnectFromTCPServer (Handle);
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp); 
	return 0;	
}
/****************************************************************6022 Functions END****************************************************************/
unsigned int CmxOpenChassis (char* IpAddress, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14]={0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	unsigned int ChassisHandle=0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = OPEN_CHASSIS;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	int tcp_error = 0;

	tcp_error = ConnectToTCPServerEx (&ChassisHandle, 55000, IpAddress, 0, 0, TIMEOUT, TCP_ANY_LOCAL_PORT);
	if (tcp_error < 0)
		memcpy(error_temp, NO_CONNECTION_ERROR, 2); 
	else
	{	
		int messageSize = 14;
		int bytesToWrite = 14;
		int bytesWritten = 0;
		while (bytesToWrite > 0)
		{
			bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
			if (bytesWritten>=0)
				bytesToWrite -= bytesWritten;
			else
				break;
		}
		if (bytesWritten < 0)
			memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
		else
		{
			unsigned int bytestoread = 14;
			int byteread = 0;
			messageSize = 14;
			int totalbytes = 0;
			while (totalbytes < 13)
			{
				byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
				if (byteread > 0)
				{
					bytestoread -= byteread;
					totalbytes = byteread + totalbytes;
				}
				else if (byteread < 0)
					break;
			}
			if ((byteread < 0) || (totalbytes < 14))
				memcpy(error_temp,TIMEOUT_ERR,2); 
			else
			{
				memcpy(crc_temp,response,10);
				memcpy(crc_array,response+10,4);
				unsigned int crc_result = CRCCALC(crc_temp,10);
				if (crc_result == deserialize_uint32(crc_array))
					memcpy(error_temp,response+8,2);
				else
					memcpy(error_temp,CRC_ERROR,2);
			}
		}	
	}
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

unsigned int CmxSetChassisIP (unsigned int ChassisHandle, unsigned int NewIpAddress[4], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = CHASSIS_SET_IP;
	TX_COMMAND[2] = (unsigned char)NewIpAddress[0];
	TX_COMMAND[3] = (unsigned char)NewIpAddress[1];
	TX_COMMAND[4] = (unsigned char)NewIpAddress[2];
	TX_COMMAND[5] = (unsigned char)NewIpAddress[3];
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	if (NewIpAddress[0] == 0)
		return ChassisHandle;
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

unsigned int CmxGetChassisIP (unsigned int ChassisHandle, unsigned int IpAddressReturn[], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = CHASSIS_GET_IP;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				unsigned char ip_array[4] = { 0x00, 0x00, 0x00, 0x00 };
				memcpy(ip_array,response+2,4);
				IpAddressReturn[0] = (unsigned int)ip_array[0];
				IpAddressReturn[1] = (unsigned int)ip_array[1]; 
				IpAddressReturn[2] = (unsigned int)ip_array[2]; 
				IpAddressReturn[3] = (unsigned int)ip_array[3]; 
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

unsigned int CmxSetChassisSubMask (unsigned int ChassisHandle, unsigned int NewSubMask[4], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = CHASSIS_SET_SUBMASK;
	TX_COMMAND[2] = (unsigned char)NewSubMask[0];
	TX_COMMAND[3] = (unsigned char)NewSubMask[1];
	TX_COMMAND[4] = (unsigned char)NewSubMask[2];
	TX_COMMAND[5] = (unsigned char)NewSubMask[3];
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

unsigned int CmxGetChassisSubMask (unsigned int ChassisHandle, unsigned int SubMaskReturn[], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = CHASSIS_GET_SUBMASK;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	 
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				unsigned char ip_array[4] = { 0x00, 0x00, 0x00, 0x00 };
				memcpy(ip_array,response+2,4);
				SubMaskReturn[0] = (unsigned int)ip_array[0];
				SubMaskReturn[1] = (unsigned int)ip_array[1]; 
				SubMaskReturn[2] = (unsigned int)ip_array[2]; 
				SubMaskReturn[3] = (unsigned int)ip_array[3]; 
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

unsigned int CmxSetChassisGateway (unsigned int ChassisHandle, unsigned int NewGateway[4], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = CHASSIS_SET_GATEWAY;
	TX_COMMAND[2] = (unsigned char)NewGateway[0];
	TX_COMMAND[3] = (unsigned char)NewGateway[1];
	TX_COMMAND[4] = (unsigned char)NewGateway[2];
	TX_COMMAND[5] = (unsigned char)NewGateway[3];
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	if (NewGateway[0] == 0)
		return ChassisHandle;
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

unsigned int CmxGetChassisGateway (unsigned int ChassisHandle, unsigned int GateWayReturn[], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = CHASSIS_GET_GATEWAY;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);

	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				unsigned char ip_array[4] = { 0x00, 0x00, 0x00, 0x00 };
				memcpy(ip_array,response+2,4);
				GateWayReturn[0] = (unsigned int)ip_array[0];
				GateWayReturn[1] = (unsigned int)ip_array[1]; 
				GateWayReturn[2] = (unsigned int)ip_array[2]; 
				GateWayReturn[3] = (unsigned int)ip_array[3]; 
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

unsigned int CmxSetModuleName (unsigned int ChassisHandle, unsigned int SlotNumber, char* ModuleName, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = CHASSIS_SET_MODULE;
	TX_COMMAND[2] = (unsigned char) SlotNumber;
	if (strcmp (ModuleName, "5010") == 0)
		TX_COMMAND[3] = CMX5010;
	else if (strcmp (ModuleName, "4010") == 0)
		TX_COMMAND[3] = CMX4010;
	else if (strcmp (ModuleName, "6010") == 0)
		TX_COMMAND[3] = CMX6010;
	else if (strcmp (ModuleName, "6012") == 0)
		TX_COMMAND[3] = CMX6012;
	else if (strcmp (ModuleName, "6012C") == 0)
		TX_COMMAND[3] = CMX6012C;
	else if (strcmp (ModuleName, "6014") == 0)
		TX_COMMAND[3] = CMX6014;
	else if (strcmp (ModuleName, "6020") == 0)
		TX_COMMAND[3] = CMX6020;
	else if (strcmp (ModuleName, "6022") == 0)
		TX_COMMAND[3] = CMX6022;
	else if (strcmp (ModuleName, "1553") == 0)
		TX_COMMAND[3] = CMX1553;	
	else if (strcmp (ModuleName, "5015") == 0)
		TX_COMMAND[3] = CMX5015;
	else if (strcmp (ModuleName, "4011") == 0)
		TX_COMMAND[3] = CMX4011;
	else
		TX_COMMAND[3] = 0;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

unsigned int CmxGetModuleName (unsigned int ChassisHandle, unsigned int SlotNumber, unsigned int *ModuleType, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = CHASSIS_GET_MODULE;
	TX_COMMAND[2] = (unsigned char) SlotNumber;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				*ModuleType = response[2]; 
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

unsigned int CmxDeploymentStatus (unsigned int ChassisHandle, unsigned int *DeploymentStatus, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = CHASSIS_GET_DEP_STATUS;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
			{
				memcpy(error_temp,response+8,2);
				*DeploymentStatus = response[2]; 
			}
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

unsigned int CmxDiscardChassis(unsigned int ChassisHandle, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char response[14] = {0};
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};	
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	int tcp_error = 0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = DISCARD_CHASSIS;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	
	int messageSize = 14;
	int bytesToWrite = 14;
	int bytesWritten = 0;
	while (bytesToWrite > 0)
	{
		bytesWritten = ClientTCPWrite (ChassisHandle, &TX_COMMAND[messageSize - bytesToWrite], bytesToWrite, 0);
		if (bytesWritten>=0)
			bytesToWrite -= bytesWritten;
		else
			break;
	}
	if (bytesWritten < 0)
		memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
	else
	{
		unsigned int bytestoread = 14;
		int byteread = 0;
		messageSize = 14;
		int totalbytes = 0;
		while (totalbytes < 13)
		{
			byteread = ClientTCPRead (ChassisHandle, &response[messageSize - bytestoread], (14 - totalbytes), TIMEOUT);
			if (byteread > 0)
			{
				bytestoread -= byteread;
				totalbytes = byteread + totalbytes;
			}
			else if (byteread < 0)
				break;
		}
		if ((byteread < 0) || (totalbytes < 14))
			memcpy(error_temp,TIMEOUT_ERR,2); 
		else
		{
			memcpy(crc_temp,response,10);
			memcpy(crc_array,response+10,4);
			unsigned int crc_result = CRCCALC(crc_temp,10);
			if (crc_result == deserialize_uint32(crc_array))
				memcpy(error_temp,response+8,2);
			else
				memcpy(error_temp,CRC_ERROR,2);
		}
	}	
	*Error = (unsigned int)deserialize_uint32(error_temp);
	tcp_error = DisconnectFromTCPServer (ChassisHandle);
	if (tcp_error < 0)
		memcpy(error_temp,GENERIC_ERROR,2);
	return 0;	
}

unsigned int CmxRebootSystem (char* IpAddress, unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};  
	unsigned int ChassisHandle=0;
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	int tcp_error = 0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = REBOOT_SYSTEM;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE);
	tcp_error = ConnectToTCPServer (&ChassisHandle, RebootPort, IpAddress, 0, 0, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,NO_CONNECTION_ERROR, 2); 
	else
	{
		tcp_error = ClientTCPWrite (ChassisHandle, TX_COMMAND, 14, TIMEOUT);
		if (tcp_error < 0)
			memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
		else
		{
			tcp_error = DisconnectFromTCPServer (ChassisHandle);
			if (tcp_error < 0)
				memcpy(error_temp,GENERIC_ERROR,2); 
		}
	}
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	waitMs();
	return ChassisHandle;
}

unsigned int CmxDebugChassis (unsigned int ChassisIP[], unsigned int *Error)
{
	unsigned char TX_COMMAND[14];
	unsigned char   *response = NULL;
 	unsigned char error_temp[4] = {0x00, 0x00, 0x00, 0x00};  
	unsigned int ChassisHandle=0;
	int RX_SIZE = 14;
	unsigned char crc_temp[10];
	unsigned int crc_u32=0;
	int tcp_error = 0;
	TX_COMMAND[0] = 0x00;
	TX_COMMAND[1] = CHASSIS_GET_IP;
	TX_COMMAND[2] = 0x00;
	TX_COMMAND[3] = 0x00;
	TX_COMMAND[4] = 0x00;
	TX_COMMAND[5] = 0x00;
	TX_COMMAND[6] = 0x00;
	TX_COMMAND[7] = 0x00;
	TX_COMMAND[8] = 0x00;
	TX_COMMAND[9] = 0x00;
	crcInit();
	memcpy(crc_temp,TX_COMMAND,10);
	crc_u32 = CRCCALC(crc_temp,10);
	unsigned char crc_array[4];
	deserialize_uint32B(crc_array,&crc_u32);
	memcpy(TX_COMMAND + 10, crc_array,4);
	response = malloc(RX_SIZE);
	tcp_error = ConnectToTCPServer (&ChassisHandle, DebugPort, "10.10.10.10", 0, 0, TIMEOUT);
	if (tcp_error < 0)
		memcpy(error_temp,NO_CONNECTION_ERROR, 2); 
	else
	{
		tcp_error = ClientTCPWrite (ChassisHandle, TX_COMMAND, 14, TIMEOUT);
		if (tcp_error < 0)
			memcpy(error_temp, COMMAND_SEND_ERROR, 2); 
		else
		{
			unsigned int bytestoread = 14;
			int byteread = 0;
			unsigned int totalbyte = 0;
			unsigned int messagesize = 14;
			clock_t start_time = clock();
			while (clock() < start_time + TIMEOUT)
			{
				byteread = ClientTCPRead (ChassisHandle, &response[messagesize - bytestoread], bytestoread, 0);
				waitMs();
				if (byteread >=0)
				{
					bytestoread -= byteread;
					totalbyte = byteread + totalbyte;
				}
				if (totalbyte == messagesize)
				break;
			}
			if (totalbyte <= 0)
				memcpy(error_temp,TIMEOUT_ERR,2); 
			else
			{
				memcpy(crc_temp,response,10);
				memcpy(crc_array,response+10,4);
				unsigned int crc_result = CRCCALC(crc_temp,10);
				if (crc_result == deserialize_uint32(crc_array))
				{
					memcpy(error_temp,response+8,2);
					unsigned char ip_array[4] = { 0x00, 0x00, 0x00, 0x00 };
					memcpy(ip_array,response+2,4);
					ChassisIP[0] = (unsigned int)ip_array[0];
					ChassisIP[1] = (unsigned int)ip_array[1]; 
					ChassisIP[2] = (unsigned int)ip_array[2]; 
					ChassisIP[3] = (unsigned int)ip_array[3]; 
				}
				else
					memcpy(error_temp,CRC_ERROR,2);
			}
		}
	}
	DisconnectTCPClient(ChassisHandle);
	free(response);
	*Error = (unsigned int)deserialize_uint32(error_temp);
	return ChassisHandle;
}

/*GetApiVersion  STM32 tarafinda kullanilmamaktadir, sadece protokol tarafinda  CVI dan gelen komutun karsiliginda islem yapilmaktadir.*/
void CmxGetApiVersion(unsigned char *Version)
{
	memcpy(Version,VERSION,3); 
}

//==============================================================================
// DLL main entry-point functions

int __stdcall DllMain (HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
	switch (fdwReason) {
		case DLL_PROCESS_ATTACH:
			if (InitCVIRTE (hinstDLL, 0, 0) == 0)
				return 0;	  /* out of memory */
			break;
		case DLL_PROCESS_DETACH:
			CloseCVIRTE ();
			break;
	}
	
	return 1;
}

int __stdcall DllEntryPoint (HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
	/* Included for compatibility with Borland */

	return DllMain (hinstDLL, fdwReason, lpvReserved);
}

int itoa(int value,char *ptr)
     {
        int count=0,temp;
        if(ptr==NULL)
            return 0;   
        if(value==0)
        {   
            *ptr='0';
            return 1;
        }

        if(value<0)
        {
            value*=(-1);    
            *ptr++='-';
            count++;
        }
        for(temp=value;temp>0;temp/=10,ptr++);
        *ptr='\0';
        for(temp=value;temp>0;temp/=10)
        {
            *--ptr=temp%10+'0';
            count++;
        }
        return count;
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

uint16_t deserialize_uint16(unsigned char *buf)
{
    uint16_t *x = (uint16_t*)buf;
    return *x;
}

uint8_t * serialize_uint16B(uint8_t *buffer, uint16_t * value)
{
    *(uint16_t*)buffer = *value;
    return buffer;
}


