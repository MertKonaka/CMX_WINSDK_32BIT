

#ifndef __GlobalVariables_H__
#define __GlobalVariables_H__

#ifdef __cplusplus
    extern "C" {
#endif

#include "cvidef.h"
//CMX1553 Protocol Commands Begin//
#define CM_OPEN_BOARD						0x01
#define CM_INIT_BOARD						0x02
#define CM_BOARD_SET_1553_MODE				0X03
#define CM_BOARD_GET_1553_MODE				0x04
#define CM_1553_RESET						0X05
#define CM_SHARED_INIT						0X06
#define CM_BOARD_RAM_TEST					0X07

#define CM_BC_INIT_MSG						0x08
#define CM_BC_UPDATE_MSG					0x09
#define CM_BC_INIT_MSG_INST_LIST			0X0A
#define	CM_BC_SET_TTAG						0x0B
#define CM_BC_GET_PROGRESS					0X0C
#define CM_BC_GET_MSG_BLOCK					0x0D
#define CM_BC_GET_MSG_DATA 					0x0E
#define CM_BC_STOP							0x0F
#define CM_BC_ENABLE						0x10
#define CM_BC_START							0X11
#define CM_BC_TRIGGER						0x12

#define CM_MT_SET_TTAG						0X13
#define CM_MT_SET_FILTER					0X14
#define CM_MT_GET_LAST_COMMAND				0X15
#define CM_MT_GET_LAST_MSG					0X16
#define CM_MT_ENABLE						0X17
#define CM_MT_STOP							0X18


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
#define CM_RESET_BOARD						0x33


#define CM_SET_IP							0X34
#define CM_GET_IP							0x35
#define CM_RESET_IP							0x36
#define CM_SET_MAC							0x37
#define CM_GET_MAC							0x38
#define CM_SET_SERIAL						0X39
#define CM_SET_FW							0X3A
#define CM_GET_FIRMWARE_VERSION				0x3B
#define CM_GET_SERIAL_NUMBER				0x3C
#define CM_DISCARD_BOARD					0X3D

//*BC MACROS//
#define ENABLE 	1
#define DISABLE 0

//1553 GET MSG DATA MACROS
#define BC_MODE_MASK	0X01
#define RT1_MODE_MASK	0X02
#define RT2_MODE_MASK	0X04
#define MT_MODE_MASK	0X08

//BC INIT MSG MACROS
#define TXTTMC17		1<<15	// only applies for mode code 17: transmit BC time tag count
#define MEMASK			1<<14	// if bit 0 = 0, Status Set occurs for RT Status Word Msg Error bit
#define SRQMASK			1<<13	// if bit 0 = 0, Status Set occurs for RT Status Word Svc Request bit
#define BSYMASK			1<<12	// if bit 0 = 0, Status Set occurs for RT Status Word Busy bit
#define SSFMASK			1<<11	// if bit 0 = 0, Status Set occurs for RT Status Word Subsystem Fail bit
#define TFMASK			1<<10	// if bit 0 = 0, Status Set occurs for RT Status Word Terminal Flag bit
#define RSVMASK			1<<9	// if bit 0 = 0, Status Set occurs for any RT Status Word Reserved bit 7-5
#define RTRYENA			1<<8	// if retry enabled in BC Config reg, retry occurs for Status Set
#define USEBUSA			1<<7	// if bit = 1 then use Bus A, if bit = 0 then use Bus B
#define USEBUSB			0<<7
#define SFTEST			1<<6	// if bit = 1 then use offline self-test
#define MSKBCR			1<<5	// if BCRME = 1 in BC Config, this bit INVERTED reflects expected BCR status,
							//	mismatch when BCR = 1 causes status set
							// if BCRME = 0 in BC Config, this bit reflects expected BCR status,
							//	mismatch = status set
#define EOMINT			1<<4	// if BCEOM interrupt is enabled, this bit causes message EOM interrupt
							// bit 3 reserved
#define MCODE			1<<2	// select mode code message format
#define BCST			1<<1	// select broadcast message format
#define RT_RT			1<<0	// select RT-to-RT message format

#define TIMESTAMP_MODE		0	//USE TIMESTAMP BETWEEN MESSAGES
#define TRIGGER_MODE		1	//USE BC TRIG PÝN FOR SEND EVERY MESSAGE

//BC INIT MSG MACROS
#define BUSA				1<<7
#define BUSB				0<<7

//BC INIT MSG DATA MACROS
#define SUBBADDR_RX_DATA	0
#define SUBBADDR_TX_DATA	1
#define BRDCST_RX_DATA		2
#define MC_RX_DATA			3

//BC INIT MSG MACROS
#define RX_MSG_CMD			0
#define TX_MSG_CMD			1
#define BRDCST_MSG_CMD  	2
#define TX_MC_MSG_CMD   	3
#define RX_MC_MSG_CMD   	4

//RT ADDRESS MACROS
#define RT_ADDR0 			0
#define RT_ADDR1 			1
#define RT_ADDR2 			2
#define RT_ADDR3 			3
#define RT_ADDR4 			4
#define RT_ADDR5 			5
#define RT_ADDR6 			6
#define RT_ADDR7 			7
#define RT_ADDR8 			8
#define RT_ADDR9 			9
#define RT_ADDR10 			10
#define RT_ADDR11 			11
#define RT_ADDR12 			12
#define RT_ADDR13 			13
#define RT_ADDR14 			14
#define RT_ADDR15 			15
#define RT_ADDR16 			16
#define RT_ADDR17 			17
#define RT_ADDR18 			18
#define RT_ADDR19 			19
#define RT_ADDR20 			20
#define RT_ADDR21 			21
#define RT_ADDR22 			22
#define RT_ADDR23 			23
#define RT_ADDR24 			24
#define RT_ADDR25 			25
#define RT_ADDR26 			26
#define RT_ADDR27 			27
#define RT_ADDR28 			28
#define RT_ADDR29 			29
#define RT_ADDR30 			30
#define RT_ADDR31 			31

//RT SUBBADDRESS MACROS
#define RT_SUBBADDR1		1
#define RT_SUBBADDR2		2
#define RT_SUBBADDR3		3
#define RT_SUBBADDR4		4
#define RT_SUBBADDR5		5
#define RT_SUBBADDR6		6
#define RT_SUBBADDR7		7
#define RT_SUBBADDR8		8
#define RT_SUBBADDR9		9
#define RT_SUBBADDR10		10
#define RT_SUBBADDR11		11
#define RT_SUBBADDR12		12
#define RT_SUBBADDR13		13
#define RT_SUBBADDR14		14
#define RT_SUBBADDR15		15
#define RT_SUBBADDR16		16
#define RT_SUBBADDR17		17
#define RT_SUBBADDR18		18
#define RT_SUBBADDR19		19
#define RT_SUBBADDR20		20
#define RT_SUBBADDR21		21
#define RT_SUBBADDR22		22
#define RT_SUBBADDR23		23
#define RT_SUBBADDR24		24
#define RT_SUBBADDR25		25
#define RT_SUBBADDR26		26
#define RT_SUBBADDR27		27
#define RT_SUBBADDR28		28
#define RT_SUBBADDR29		29
#define RT_SUBBADDR30		30

#define NO_MC				0

//BC INIT MSG LIST
#define LOOP_ENABLE			1
#define LOOP_DISABLE		0

//MSG GAP TIME
#define MS_1				0x000A
#define MS_10				0x0064
#define MS_100				0x03E8
#define MS_1000				0x2710

//BCTTAG MACROS
#define HIGH_RES			1		//High Resolution (32 bit)
#define LOW_RES				0		//Low Resolution (16 bit)

#define TTAG_2U				2<<0	// BC & RT time tag counter uses internally generated 2us clock
#define TTAG_4U				3<<0	// BC & RT time tag counter uses internally generated 4us clock
#define TTAG_8U				4<<0	// BC & RT time tag counter uses internally generated 8us clock
#define TTAG_16U			5<<0	// BC & RT time tag counter uses internally generated 16us clock
#define TTAG_32U			6<<0	// BC & RT time tag counter uses internally generated 32us clock
#define TTAG_64U			7<<0	// BC & RT time tag counter uses internally generated 64us clock



//*MT MACROS//

//MT TTAG MACROS
#define MTTAG_2U			2<<4	// MT time tag counter uses internally generated 2us clock
#define MTTAG_4U			3<<4	// MT time tag counter uses internally generated 4us clock
#define MTTAG_8U			4<<4	// MT time tag counter uses internally generated 8us clock
#define MTTAG_16U			5<<4	// MT time tag counter uses internally generated 16us clock
#define MTTAG_32U			6<<4	// MT time tag counter uses internally generated 32us clock
#define MTTAG_64U			7<<4	// MT time tag counter uses internally generated 64us clock
#define MTTAG_100N			8<<4	// MT time tag counter uses internally generated 100ns clock

//*RT MACROS//
#define RT1						1
#define RT2						2


#define SUPPORT_BRDCST			1		//Commands to RT31 are valid
#define NOSUPPORT_BRDCST		0		//Commands to RT31 are unvalid
#define UNDEF_MCODES_VALID		1		//Undefined & reserved mode commands are valid
#define UNDEF_MCODES_NOVALID	0		//Undefined & reserved mode commands are unvalid
#define USE_SMCP				1		//Mode command results are stored in descriptor table itself,not in assigned RAM buffer
#define DONT_USE_SMCP			0		//Descriptor table is initialized to store mode command results in assigned RAM buffer
#define ILLEGAL_CMD_DETECT		1		//"in form" response for all valid or unvalid commands
#define ILLEGAL_CMD_NOT_DETECT	0		//"in form" response for all valid commands

//END OF 1553/

int Declare_Your_Functions_Here (int x);

#ifdef __cplusplus
    }
#endif

#endif  
